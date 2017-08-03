__author__ = 'gleicher'

import numpy as N
import rank

def clipMatrix(matrix, minPerRow=1, minPerCol=1):
    """
    zeros out elements of a matrix such that only the highest values in
    each row column are left
    guarantees at least minPerRow (or Col) - but in the case of ties, it may include
    more (it uses rank.rank in order to handle ties correctly)
    also, a row may include many elements if this is required to keep the columns happy
        (and vice versa)
    :param matrix:
    :param minPerRow:
    :param minPerCol:
    :return:
    """
    m = N.zeros(matrix.shape)
    if minPerRow:
        for j,r in enumerate(matrix):    # each row
            a = rank.rank(r,ties="max")
            lastIdx = len(r)-minPerRow
            for i in range(len(r)):
                if a[i] >= lastIdx:
                    m[j,i] = 1
    if minPerCol:
        for j,c in enumerate(matrix.transpose()): # each column
            a = rank.rank(c,ties="max")
            lastIdx = len(c)-minPerCol
            for i in range(len(c)):
                if a[i] >= lastIdx:
                    m[i,j] = 1
    m = m * matrix
    return m

def mThresh(matrix,dir="both"):
    """
    find a threshold such that every row/column still has 1 entry
    :param matrix:
    :param minPerRowCol: minimum number of entries greater than thresh
    :return: the threshold
    """
    rowMax = min([max(r) for r in matrix])
    colMax = min([max(c) for c in matrix.transpose()])
    if dir=="both":
        one = min(rowMax,colMax)
    elif dir=="col":
        one = colMax
    elif dir=="row":
        one = rowMax
    elif dir=="either":
        one = max(rowMax,colMax)
    else:
        raise ValueError("Bad Direction to mThresh")
    return one

def rankInRows(matrix, reverse=True, frank=False):
    """
    perform a rank transform on each ROW of the matrix
    so each row ranks its columns

    :param matrix:
    :param frank: floating point ranking tolerance for ties
    :return: an integer matrix
    """
    rmat = N.zeros(matrix.shape,dtype='int32')
    for i in range(matrix.shape[0]):
        r = matrix[i]
        n = rank.rank(r,ties="max",frank=frank)
        rmat[i] = N.max(n) - n if reverse else n
    return rmat

##############################################################################
## test matrix generation
### some sample matrices for testing
def band(n,bw=1):
    """
    create a banded matrix with recipricol decaying bands (1 at diagonal, 1/2, ...)
    :param n:
    :param bw:
    :return:
    """
    m = N.eye(n)
    for bb in range(bw):
        b = bb+1
        for i in range(len(m)-b):
            m[i+b,i] = 1.0/(b+1.0)
            m[i,i+b] = 1.0/(b+1.0)
    return m

def block(n,bs=3):
    """
    create a block diagonal matrix (of all 1s)

    :param n:
    :param bs:
    :return:
    """
    m = N.eye(n)
    for b in range(n/bs):
        top = min(n,(b+1)*bs)
        m[b*bs:top, b*bs:top] = 1
    return m

def ablock(r,c,rb=2,cb=3):
    """
     create a block diagonal matrix with rectangular (not square) blocks
    :param r:
    :param c:
    :param rb:
    :param cb:
    :return:
    """
    m = N.zeros((r,c))
    for b in range(r/rb):
        top = min(r,(b+1)*rb)
        topc = min(c, (b+1)*cb)
        m[b*rb:top, b*cb:topc] = 1
    return m
