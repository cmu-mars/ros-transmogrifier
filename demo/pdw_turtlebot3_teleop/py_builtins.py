try:
	import __builtin__ as builtins
except ImportError:
	import builtins

def list(*args, **kwargs):
	return builtins.list(*args, **kwargs)

def len(l):
	return builtins.len(l)
