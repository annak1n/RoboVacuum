import time

from rpcserver import RPC


# start RPC server
rpc = RPC()


# call a function on core number 3 
# the function is defined in test.py

while 1 :

	# invoke MyClass.rfoo() on core 2 and print the reply
	something = rpc.ssend(2,{"module":"MyClass","function":"rfoo","p1":1234}) 
	print "got retval", something


	time.sleep(1)

	# invoke MyClass.foo() on core 3
	rpc.send(3,{"module":"MyClass","function":"foo","p1":1.0})

	time.sleep(1)


	# invoke MyClass.rfoo() on this core
	something = rpc.ssend(0,{"module":"MyClass","function":"rfoo","p1":1.0})
	print "got retval", something

	time.sleep(1)

		
	# invoke MyClass.poofoo()  on core 1
#	rpc.send(1,{"module":"MyClass","function":"poofoo","p1":255})


	# check that getcore returns the right value on each core
#	if rpc.ssend(0,{"module":"RPC","function":"getcore"}) != 0 :
#		print "Error getcore 0 failed"

#	if rpc.ssend(1,{"module":"RPC","function":"getcore"}) != 1 :
#		print "Error getcore 1 failed"

#	if rpc.ssend(2,{"module":"RPC","function":"getcore"}) != 2 :
#		print "Error getcore 3 failed"

#	if rpc.ssend(3,{"module":"RPC","function":"getcore"}) != 3 :
#		print "Error getcore 3 failed"



	time.sleep(1)
