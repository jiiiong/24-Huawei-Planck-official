


RELEASE = False

if RELEASE:
    TYPE_CHECKING = False
else:
    from typing import TYPE_CHECKING

N = 200