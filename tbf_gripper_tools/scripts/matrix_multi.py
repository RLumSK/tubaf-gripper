#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TU Bergakademie Freiberg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

class StrMatrix(object):

    def __init__(self, literal="A", shape=(4,4)):
        """
        Default constructor
        :param literal: literal of the matrix, eg A
        :type literal: string
        """
        self.a = str(literal).lower()
        self.matrix = []
        self.shape = shape
        for row in range(0, shape[0]):
            self.matrix.append([])
            for col in range(0, shape[1]):
                self.matrix[row].append([[[self.a + "_{"+str(row+1)+str(col+1)+"}"]]])

    def __str__(self):
        """
        matrix to string
        :return: matrix as string
        :rtype: str
        """
        retStr = ""
        for row in range(0,self.shape[0]):
            retStr += ""
            for col in range(0,self.shape[1]):
                for summand in range(0, len(self.matrix[row][col])):
                    for factor in range(0, len(self.matrix[row][col][summand])):
                        item = self.matrix[row][col][summand][factor]
                        retStr+= item[0]
                    retStr += "+"
                retStr = retStr[:-1]  #  remove last '+'
                retStr += " & "
            retStr = retStr[:-3]
            retStr += "\\\\\n"
        retStr = retStr[:-3]
        retStr += ""
        return retStr

    def as_negativ(self, row, col):
        """
        Return negative summands
        :param row: row in the matrix
        :type row: int
        :param col: column in the matrix
        :type col: int
        :return: list of negative summands
        :rtype: list
        """
        retLst = self.matrix[row][col]
        for i in range(0, len(retLst)):
            first_factor = retLst[i][0][0]
            if first_factor[0] is not "-":
                retLst[i][0][0] = "-"+retLst[i][0][0]
            else:
                retLst[i][0][0] = retLst[i][0][0][1:]
        return retLst

    @staticmethod
    def multi(L, R):
        """
        Multiply two matrices
        :param L: left matrix
        :type L: StrMatrix
        :param R: right matrix
        :type R: StrMatrix
        :return: result
        :rtype: StrMatrix
        """
        if L.shape != R.shape:
            return None
        retMatrix = d
        for row in range(0, L.shape[0]):
            for col in range(0, L.shape[1]):
                retMatrix.matrix[row][col] = []
                for i in range(0, L.shape[1]):
                    retMatrix.matrix[row][col].extend(StrMatrix.multi_cell(L.matrix[row][i], R.matrix[i][col]))
        return retMatrix

    @staticmethod
    def sub(L, R):
        """
        Substract two matrices
        :param L: left matrix
        :type L: StrMatrix
        :param R: right matrix
        :type R: StrMatrix
        :return: result
        :rtype: StrMatrix
        """
        if L.shape != R.shape:
            return None
        retMatrix = StrMatrix(literal="S", shape=L.shape)
        for row in range(0, L.shape[0]):
            for col in range(0, L.shape[1]):
                retMatrix.matrix[row][col] = []
                retMatrix.matrix[row][col] = L.matrix[row][col]
                retMatrix.matrix[row][col].extend(R.as_negativ(row, col))
        return retMatrix

    @staticmethod
    def multi_cell(summe1, summe2):
        """
        multiply two sums given as list of summands and factors: abc+edf -> [[a,b,c],[e,d,f]]
        :param summe1:
        :type summe1:
        :param summe2:
        :type summe2:
        :return:
        :rtype:
        """
        retLst=[]
        for summand_2 in summe2:
            for summand_1 in summe1:
                retLst.extend([summand_1+summand_2])
        return retLst

if __name__ == '__main__':
    E = StrMatrix(literal="E")
    F = StrMatrix(literal="F")
    C = StrMatrix(literal="C")
    O = StrMatrix(literal="O")
    Q = StrMatrix(literal="Q")

    L = StrMatrix.multi(E, C)
    R = StrMatrix.multi(C, Q)

    M = StrMatrix.multi(L, O)
    S = StrMatrix.multi(F, R)

    G = StrMatrix.sub(M, S)

    print G

