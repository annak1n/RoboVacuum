import time

from rpcserver import RPC 


# start RPC server
rpc = RPC()

# call a function on core number 3 
# the function is defined in test.py

while 1 :

	# format an RPC request
	msg 		= {}
	msg["module"]	= "MyClass"
	msg["function"]	= "foo"
	msg["p1"] 	= 12345

	# invoke it on core 3
	rpc.send( 3, msg )

	# snoose for a while then loop
	print "doing nothing"
	time.sleep(5)
	



