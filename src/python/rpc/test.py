# Example class and method that can be invoked remotely (i.e. on another core)
# methods that you wish to invoke remotely must be expliciyly imported in rpcserver.py

class MyClass:


	# foo() - a silly method that prints its name and parameters
	# the parameters to all RPC methods are passed in a dict

	def foo(self,  po):
		print "In module " + po["module"]
		print "In function "+ po["function"]
		print "p1 = "+ str(po["p1"])



	# rfoo() - a method that returns something
	def rfoo(self, po):
		print "In module " + po["module"]
		print "In function "+ po["function"]
		print "p1 = "+ str(po["p1"])
		return 123456789		


