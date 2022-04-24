# from attr import has


# class TestClass:
#     def test_one(self):
#         x = "this"
#         assert "h" in x 

#     def test_two(self):
#         x = "hello"
#         assert hasattr(x, "check")

# from re import X
import pytest 

def f():
    raise SystemExit(1)

def test_mytest():
    with pytest.raises(SystemExit):
        f()

class TestClass:
    def test_one(self):
        x = "this"
        assert "h" in x

    def test_two(self):
        x = "hello"
        assert hasattr(x, "hello") # Check to see if object x contains the 
                                   # attribute called "hello"

    