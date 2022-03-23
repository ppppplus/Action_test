#!/usr/bin/env python3

class MyPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __hash__(self):
        return hash(MyPoint)
    def __cmp__(self, other):
        # if self.x == other.x and self.y == other.y:
        #     return True
        # else:
        #     return False
        return cmp(self.__hash__(), other.__hash__())

a1 = MyPoint(1,1)
map_a1 = MyPoint(3,3)
a2 = MyPoint(2,2)
map_a2 = MyPoint(4,4)
d = {a1:'map1', a2:'map2'}
at = MyPoint(1,1)
print(d[at)