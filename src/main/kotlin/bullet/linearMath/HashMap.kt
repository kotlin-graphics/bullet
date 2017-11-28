/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package bullet.linearMath

import bullet.i

/** very basic hashable string implementation, compatible with HashMap */
class HashString(val string: String) {

    val hash = run {
        /* Fowler / Noll / Vo (FNV) Hash */
        var hash = initialFNV
        for (i in 0 until string.length) {
            hash = hash xor (string[i].i and 0xff) /* xor  the low 8 bits */
            hash *= fnvMultiple  /* multiply by the magic number */
        }
        hash
    }

    override fun equals(other: Any?) = other is HashString && string == other.string
    override fun hashCode() = 31 * string.hashCode() + hash

    companion object {
        /* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */
        val initialFNV = 2166136261.i
        val fnvMultiple = 16777619
    }
}

val HASH_NULL = 0xffffffff.i

data class HashInt(var uid: Int = 0) {
    //to our success
    val hash: Int
        get() {
            var key = uid
            // Thomas Wang's hash
            key += (key shl 15).inv()
            key = key xor (key ushr 10)
            key += key shl 3
            key = key xor (key ushr 6)
            key += (key shl 11).inv()
            return key xor (key ushr 16)
        }
}