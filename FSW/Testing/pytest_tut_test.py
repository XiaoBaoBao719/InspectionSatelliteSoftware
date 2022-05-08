import pytest
import os
import sys
import math

def func(x):
    return x + 1

def test_answer():
    assert func(3) == 5