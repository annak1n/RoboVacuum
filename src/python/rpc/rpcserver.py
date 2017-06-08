import socket, select
import netstring
import json
import thread
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

####################################################
# RPC - container class for the server and client 
####################################################
class RPC:

	# sockets for reaching other servers
	s=[0,0,0,0]

	# constructor starts a server on this core and then 
        # connects to servers on all the other cores in the core list
	def __init__(self) :

		# discover which core I am running on
		p = psutil.Process()
		globals()["thiscore"] = p.cpu_affinity()[0] 

		# start a server on this core
		thread.start_new_thread(rpc_server_thread,(0,''))

		for c in coreList :
			# not this core ?
			if c != thiscore :
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

				

	# send() - invoke an RPC on another core
	def send(self, core, msg) :

		# encode the python dict as JSON
		js = json.dumps(msg)

		# format it as a netstring
		ns = str(len(js))+":"+js+","

		# write it to the socket that connects the destination core
		self.s[core].send(ns)




##############################################################################################
# rpc_server_thread() - this server thread is run on each core, it accepts connections from
# other cores, receives RCP messages decodes them and invokes the appropriate function
##############################################################################################
def rpc_server_thread(argc, argv) :

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
							
									# each net string is a JSON coded dict	
									# convert from JSON to a python dict
									po = json.loads(p)

									# extract the module and function name
									mname = po["module"] 
									fname = po["function"]
									try:
										# Invoke the function
										# pass the dict to the function so that it
										# can extract its own parameters	
										getattr(globals()[mname](),fname)(po)
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




