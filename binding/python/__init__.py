# This file is part of PG.
#
# PG is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# PG is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with PG.  If not, see <http://www.gnu.org/licenses/>.

# Taken from https://github.com/roboptim/roboptim-core-python/blob/master/src/roboptim/core/__init__.py#L11-L17
# Here, we use RTLD_GLOBAL to link with roboptim-core since the Python module
# is a plugin, itself calling RobOptim solver plugins. Without this, the Python
# plugin cannot access local symbols of roboptim-core. This is not ideal, but
# at least plugins do not need to link with roboptim-core themselves. A better
# solution may be implemented later on.
from ctypes import CDLL, RTLD_GLOBAL
CDLL("libroboptim-core.so", RTLD_GLOBAL)

from _pg import *

