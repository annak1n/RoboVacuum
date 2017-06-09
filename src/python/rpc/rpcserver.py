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


# baseport - the base TCP port number for the servers, the CPU core number
# is added to this to obtain the unique port number for each server.
baseport = 4320  


# rxbytesmax - the number of bytes to read from the socket at one go
# can be tuned, not clear yet how it will impact performance
rxbytesmax = 20


# RPC message types
ASYNC_REQ = 0
SYNC_REQ  = 1
RETVAL    = 2		

thisrpcserver = ''


####################################################
# RPC - container class for the server and client 
####################################################
class RPC:

	# sockets for reaching other servers
	s=[0,0,0,0]

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

		for c in coreList :
			# create a socket
			self.s[c] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

			# try to connect
			while 1 :
				try:
					self.s[c].connect(("127.0.0.1",baseport+c))
					break
				except socket.error as e:
					if e.errno == 111 :
						print "core",thiscore,"waiting to connect to core ",c
						time.sleep(1)


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

		# write it to the socket that connects the destination core
		self.s[dest].send(ns)

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

		# write it to the socket that connects the destination core
		self.s[dest].send(ns)

		# read the response from the socket 
		while 1 :
			try:
       				data = self.s[dest].recv(rxbytesmax)
				if data != '' :
					# see if there is enough to decode a netstring
					for p in decoder.feed(data):

						# convert from JSON to a python dict
						rsp = json.loads(p)
						if rsp["type"] == RETVAL:
							return rsp["retval"]
						else:
							print "Error response is not a RETVAL"
			except :
				pass


	###
	# thiscore() - return the core number that the server is running on
	#########
	def getcore(self) :
		return thiscore


##############################################################################################
# rpc_server_thread() - this server thread is run on each core, it accepts connections from
# other cores, receives RCP messages decodes them and invokes the appropriate function
##############################################################################################
def rpc_server_thread(server,) :

	# init the netstring decoder
	decoder = netstring.Decoder()

	# create master socket
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind(('0.0.0.0', baseport+thiscore))
	s.listen(1)
	s.setblocking(0)

	print "core",thiscore,"listening on port",baseport+thiscore

	# register the socket to be handled by epoll
	epoll = select.epoll()
	epoll.register(s.fileno(), select.EPOLLIN)

	try:
		# wait for events on the master socket
   		peers = {};
   		while True:
      			events = epoll.poll(1)
      			for i, e in events:

				# events on the master socket are the initial incomming connections 
         			if i == s.fileno():
            				try:
						# accept connections and register then for epoll
               					while True:
                  					conn, addr = s.accept()
							conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                  					conn.setblocking(0)
                  					epoll.register(conn.fileno(), select.EPOLLIN)
                  					peers[conn.fileno()] = conn
            				except socket.error:
               					pass

				# a connection has received something
         			elif e & select.EPOLLIN:
            				try:
						while 1 :
							# read some data from the socket
        						data = peers[i].recv(rxbytesmax)
							if data != '' :
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
											rsp["dest"]	= i
											rsp["retval"]	= r
	
											# encode the python dict as JSON
											js = json.dumps(rsp)

											# format it as a netstring
											ns = str(len(js))+":"+js+","

											# reply on the originating connection
											peers[i].send(ns)

									except KeyError, e:
										# The function was undefined
										print e

					# if there is nothing left to read we end up here
            				except socket.error:
               					pass

				# the connection has dropped
        			elif e & select.EPOLLHUP:
            				epoll.unregister(fileno)
            				connections[fileno].close()
            				del connections[fileno]

	# clean up and exit on all other errors
	finally:
   		epoll.unregister(s.fileno())
		epoll.close()

