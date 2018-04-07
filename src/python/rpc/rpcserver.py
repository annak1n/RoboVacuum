import socket, select
import netstring
import json
import thread, threading
import psutil
import time


# list here all classes that you wish to be able to access remotely
from test import MyClass


# coreList - list of cores that will run RPC servers
# invoke RPCs on each other
coreList = [0,1,2,3]

# thiscore - the current core number, each core may run only one python process and
# must set its core number by lauching with taskset -c 
# the actual value will initialized by the server constructor on each core 
thiscore = 0


# baseport - the base UDP port number for the servers, the CPU core number
# is added to this to obtain the unique port number for each server.
baseport = 4320  


# rxbytesmax - the number of bytes to read from the socket at one go
# can be tuned, not clear yet how it will impact performance
rxbytesmax = 4096 


# RPC message types
ASYNC_REQ = 0
SYNC_REQ  = 1
RETVAL    = 2		

thisrpcserver = ''


####################################################
# RPC - container class for the server and client 
####################################################
class RPC:

        ###
	# constructor starts a server on this core and then 
        # connects to servers on all the other cores in the core list
	#########
	def __init__(self) :

		if globals()["thisrpcserver"] :
			return

		globals()["thisrpcserver"] = self

		# discover which core I am running on
		p = psutil.Process()
		globals()["thiscore"] = p.cpu_affinity()[0] 

		# start a server on this core
		thread.start_new_thread(rpc_server_thread,(self,))


 
	###
	# send() - invoke an RPC on another core
	#########
	def send(self, dest, req) :

		msg={}
		msg["src"] 	= thiscore
		msg["dest"] 	= dest
		msg["type"]	= ASYNC_REQ
		msg["req"]	= req
	
		# encode the python dict as JSON
		js = json.dumps(msg)

		# format it as a netstring
		ns = str(len(js))+":"+js+","

		# write it to the destination core, let the server close the connection
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		addr = ('localhost',baseport+dest)
		s.sendto(ns,addr)


	###
	# ssend() - synchronous version of send that waits for a return value
	#########
	def ssend(self, dest, req) :

		# init the netstring decoder
		decoder = netstring.Decoder()

		msg = {}
		msg["src"] 	= thiscore
		msg["dest"] 	= dest
		msg["type"]	= SYNC_REQ
		msg["req"]	= req

		# encode the python dict as JSON
		js = json.dumps(msg)

		# format it as a netstring
		ns = str(len(js))+":"+js+","

		# write it to the destination core
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		addr = ('localhost',baseport+dest)
		s.sendto(ns,addr)


		# read the response from the socket 
		data,addr = s.recvfrom(rxbytesmax)
		print "got response"
		for p in decoder.feed(data):

			# convert from JSON to a python dict
			rsp = json.loads(p)
			if rsp["type"] == RETVAL:
				return rsp["retval"]
			else:
				print "Error response is not a RETVAL"


	###
	# thiscore() - return the core number that the server is running on
	#########
	def getcore(self) :
		return thiscore


##############################################################################################
# rpc_server_thread() - this server thread is run on each core, it accepts messages from
# other cores, receives RCP messages decodes them and invokes the appropriate function
##############################################################################################
def rpc_server_thread(server,) :

	# init the netstring decoder
	decoder = netstring.Decoder()

	# create socket
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.bind(('0.0.0.0', baseport+thiscore))

	print "core",thiscore,"listening on port",baseport+thiscore


	while 1:
		data, addr = s.recvfrom(rxbytesmax)

		# see if there is enough to decode a netstring
		for p in decoder.feed(data):
							
			# each request in a net string is a JSON coded dict	
			# convert from JSON to a python dict
			msg = json.loads(p)

			# extract the module and function name
			req = msg["req"]
			mname = req["module"] 
			fname = req["function"]
			try:
														# Invoke the function
				# pass the dict to the function so that it
				# can extract its own parameters	
				r = getattr(globals()[mname](),fname)(req)

				# if there is a return value format a response
				# and send it back
				if msg["type"] == SYNC_REQ:
					if r == '' :
						print "Error no return value"
					rsp = {}
					rsp["module"]	= mname
					rsp["function"]	= fname	
					rsp["type"]	= RETVAL
					rsp["src"]	= thiscore
					rsp["dest"]	= msg["src"] 
					rsp["retval"]	= r
	
					# encode the python dict as JSON
					js = json.dumps(rsp)

					# format it as a netstring
					ns = str(len(js))+":"+js+","

					# reply on the originating connection
					s.sendto(ns,addr)

			except KeyError, e:
				# The function was undefined
				print e

