# Example of a class and methods that can be invoked remotely (i.e. on another core)
# methods that you wish to invoke remotely must be expliciyly imported in rpcserver.py


def trace (core, po) :
	print "----------------------------"
	print "Core ",core
	print "In module " + po["module"]
	print "In function "+ po["function"]
	print "p1 = "+ str(po["p1"])



class MyClass:

	# foo() - a silly method that prints its name and parameters
	def foo(self,  po):
		trace(getcore(),po)


	# rfoo() - a method that returns something
	def rfoo(self, po):
		trace(getcore(),po)
		return 123456789		


	# poofoo() - a methof that invokes foo() on core 2
	def poofoo(self, po) :
		trace(getcore(),po)
		from rpcserver import RPC
		rpc = RPC()
		rpc.send(2,{"module":"MyClass","function":"foo","p1":3.14259})


# obtain the current CPU core
def getcore() :
	from rpcserver import RPC
	rpc = RPC()
	return rpc.getcore()







