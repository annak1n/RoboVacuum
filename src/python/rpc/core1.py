from rpcserver import RPC
import time

# start RPC server
rpc = RPC()


# workload goes here
while 1 :
	print "core ",rpc.getcore()," alive"
	time.sleep(5)




