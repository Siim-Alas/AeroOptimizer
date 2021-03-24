
#pragma once

namespace AeroOptimizer
{
	namespace LinearAlgebra
	{
		/*
		* result = left + right, adds the left and right vectors.
		*
		* @param left, the array representing one vector to be added.
		* @param right, the array representing the other vector to be added.
		* @param result, the array where the resulting vector gets saved.
		* @param n, the amount of elements in each vector (their dimension).
		*/
		void AddVectors(const double* left, const double* right, double* result, int n);
		/*
		* result = left x right, computes cartesian 3D the cross product of the left and right vector.
		*
		* @param left, the array of length 3 representing the left vector.
		* @param right, the array of length 3 representing the right vector.
		* @param result, the array of length 3 where the results will get written.
		*/
		void CrossProduct(const double* left, const double* right, double* result);
		/*
		* v1 dot v2, calculates the dot product (in cartesian coordinates) of two vectors.
		*
		* @param v1, a pointer to the first array representing a vector.
		* @param v2, a pointer to the second array representing a vector.
		* @param n, the number of components in each vector.
		*
		* @return The dot product of the two vectors.
		*/
		double DotProduct(const double* v1, const double* v2, int n);
		/*
		* Mv, multiplies a square matrix and a vector together. This assumes that both the matrix
		* and vector have the specified number of dimensions.
		*
		* @param M, a pointer to the 2-dimensional array representing the matrix.
		* @param v, a pointer to the 1-dimensional array representing the vector.
		* @param result, the resulting vector (with n components).
		* @param n, the number of elements the vector has and the "side length" of the matrix.
		*
		* @return A pointer to the resulting vector.
		*/
		void MatrixVectorMult(const double* M, const double* v, double* result, int n);
		/*
		* result = left - right, subtracts the right vector from the left one.
		* 
		* @param left, the array representing the vector from which to subtract the other vector.
		* @param right, the array representing the vector to subtract from the other vector.
		* @param result, the array where the resulting vector gets stored.
		* @param n, the amount of elements in each vector (their dimension).
		*/
		void SubtractVectors(const double* left, const double* right, double* result, int n);
	};
}
