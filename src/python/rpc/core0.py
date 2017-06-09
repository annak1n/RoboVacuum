import time

from rpcserver import RPC


# start RPC server
rpc = RPC()


# call a function on core number 3 
# the function is defined in test.py

while 1 :

	# format an async RPC request
	msg = {}
	msg["module"]	= "MyClass"
	msg["function"]	= "foo"
	msg["p1"] 	= 12345

	# invoke it on core 3
	rpc.send( 3, msg )

	time.sleep(1)

	something = rpc.ssend(2, msg)
	print "got retval", something


	time.sleep(1)
	
	rpc.send(3,{"module":"MyClass","function":"foo","p1":1.0})

	time.sleep(1)

	something = rpc.ssend(0,{"module":"MyClass","function":"foo","p1":1.0})
	print "got retval", something

	time.sleep(1)

	rpc.send(1,{"module":"MyClass","function":"poofoo","p1":255})

	time.sleep(1)
