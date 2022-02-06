﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
    public bool launched = false;
    Vector3[] X;                                  // world position
    Vector3[] Y;								  // temp world position
    Vector3[] Q;								  // local coordinate containing ri
    Vector3[] V;								  // speed
    Matrix4x4 QQt = Matrix4x4.zero;
    Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
	float linear_decay = 0.999f;                  // for velocity decay
	Vector3 ground = new Vector3(0, 0.01f, 0);    // position of the ground for collision
    Vector3 groundNormal = new Vector3(0, 1, 0);  // normal of the ground for collision
    Vector3 wall = new Vector3(2.01f, 0, 0);      // position of the wall for collision
    Vector3 wallNormal = new Vector3(-1, 0, 0);   // normal of the wall for collision
    float a = 0.0f;                               // define large friction where a is 0, only static friction
    float mu_N = 5.0f;							  // μ_N coefficient of Restitution


    // Start is called before the first frame update
    void Start()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		V = new Vector3[mesh.vertices.Length];
		X = mesh.vertices;
		Y = mesh.vertices;
		Q = mesh.vertices;

		//Centerizing Q.
		Vector3 c = Vector3.zero;
		for (int i = 0; i < Q.Length; i++)
			c += Q[i];
		c /= Q.Length;
		Debug.Log("C: " + c);
		for (int i = 0; i < Q.Length; i++)
			Q[i] -= c;

		//Get QQ^t ready.
		for (int i = 0; i < Q.Length; i++)
		{
			QQt[0, 0] += Q[i][0] * Q[i][0];
			QQt[0, 1] += Q[i][0] * Q[i][1];
			QQt[0, 2] += Q[i][0] * Q[i][2];
			QQt[1, 0] += Q[i][1] * Q[i][0];
			QQt[1, 1] += Q[i][1] * Q[i][1];
			QQt[1, 2] += Q[i][1] * Q[i][2];
			QQt[2, 0] += Q[i][2] * Q[i][0];
			QQt[2, 1] += Q[i][2] * Q[i][1];
			QQt[2, 2] += Q[i][2] * Q[i][2];
		}
		QQt[3, 3] = 1;

		for (int i = 0; i < X.Length; i++)
			V[i][0] = 4.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position = Vector3.zero;
		transform.rotation = Quaternion.identity;
	}

	Matrix4x4 vector3x1dotvector1x3(Vector3 A, Vector3 B)
	{
		Matrix4x4 rlt = Matrix4x4.zero;
		rlt[3, 3] = 1.0f;

		rlt[0, 0] = A[0] * B[0];
		rlt[0, 1] = A[0] * B[1];
		rlt[0, 2] = A[0] * B[2];

		rlt[1, 0] = A[1] * B[0];
		rlt[1, 1] = A[1] * B[1];
		rlt[1, 2] = A[1] * B[2];

		rlt[2, 0] = A[2] * B[0];
		rlt[2, 1] = A[2] * B[1];
		rlt[2, 2] = A[2] * B[2];

		return rlt;
	}

	// Polar Decomposition that returns the rotation from F.
	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					C[ii, jj] += F[kk, ii] * F[kk, jj];

		Matrix4x4 C2 = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					C2[ii, jj] += C[ii, kk] * C[jj, kk];

		float det = F[0, 0] * F[1, 1] * F[2, 2] +
						F[0, 1] * F[1, 2] * F[2, 0] +
						F[1, 0] * F[2, 1] * F[0, 2] -
						F[0, 2] * F[1, 1] * F[2, 0] -
						F[0, 1] * F[1, 0] * F[2, 2] -
						F[0, 0] * F[1, 2] * F[2, 1];

		float I_c = C[0, 0] + C[1, 1] + C[2, 2];
		float I_c2 = I_c * I_c;
		float II_c = 0.5f * (I_c2 - C2[0, 0] - C2[1, 1] - C2[2, 2]);
		float III_c = det * det;
		float k = I_c2 - 3 * II_c;

		Matrix4x4 inv_U = Matrix4x4.zero;
		if (k < 1e-10f)
		{
			float inv_lambda = 1 / Mathf.Sqrt(I_c / 3);
			inv_U[0, 0] = inv_lambda;
			inv_U[1, 1] = inv_lambda;
			inv_U[2, 2] = inv_lambda;
		}
		else
		{
			float l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
			float k_root = Mathf.Sqrt(k);
			float value = l / (k * k_root);
			if (value < -1.0f) value = -1.0f;
			if (value > 1.0f) value = 1.0f;
			float phi = Mathf.Acos(value);
			float lambda2 = (I_c + 2 * k_root * Mathf.Cos(phi / 3)) / 3.0f;
			float lambda = Mathf.Sqrt(lambda2);

			float III_u = Mathf.Sqrt(III_c);
			if (det < 0) III_u = -III_u;
			float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2 * III_u / lambda);
			float II_u = (I_u * I_u - I_c) * 0.5f;


			float inv_rate, factor;
			inv_rate = 1 / (I_u * II_u - III_u);
			factor = I_u * III_u * inv_rate;

			Matrix4x4 U = Matrix4x4.zero;
			U[0, 0] = factor;
			U[1, 1] = factor;
			U[2, 2] = factor;

			factor = (I_u * I_u - II_u) * inv_rate;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					U[i, j] += factor * C[i, j] - inv_rate * C2[i, j];

			inv_rate = 1 / III_u;
			factor = II_u * inv_rate;
			inv_U[0, 0] = factor;
			inv_U[1, 1] = factor;
			inv_U[2, 2] = factor;

			factor = -I_u * inv_rate;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					inv_U[i, j] += factor * U[i, j] + inv_rate * C[i, j];
		}

		Matrix4x4 R = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					R[ii, jj] += F[ii, kk] * inv_U[kk, jj];
		R[3, 3] = 1;
		return R;
	}

	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
	{
		for (int i = 0; i < Q.Length; i++)
		{
			Vector3 x = (Vector3)(R * Q[i]) + c;

			V[i] = (x - X[i]) * inv_dt;
			X[i] = x;
		}
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices = X;
	}

	void Collision(float inv_dt)
	{
		for (int i = 0; i < Q.Length; i++)
		{
			if (Vector3.Dot(X[i] - ground, groundNormal) < 0 && Vector3.Dot(V[i], groundNormal) < 0)  // collision with ground
			{
				Vector3 vN = Vector3.Dot(V[i], groundNormal) * groundNormal;
				// Does Not consider vT here, a is 0, only static friction
				V[i] = -mu_N * vN;
			}
			else if (Vector3.Dot(X[i] - wall, wallNormal) < 0 && Vector3.Dot(V[i], wallNormal) < 0)  // collision with wall
			{
				Vector3 vN = Vector3.Dot(V[i], wallNormal) * wallNormal;
				// Does Not consider vT here, a is 0, only static friction
				V[i] = -mu_N * vN;
			}
		}
	}

	// Update is called once per frame
	void Update()
	{
		if (Input.GetKey("r"))
		{
			launched = false;
			for (int i = 0; i < V.Length; i++)
			{
				V[i] = new Vector3(4.0f, 0.0f, 0.0f);
			}

			Update_Mesh(new Vector3(0, 0.6f, 0), Matrix4x4.Rotate(transform.rotation), 0);
		}

		if (Input.GetKey("l"))
		{
			launched = true;
			for (int i = 0; i < V.Length; i++)
			{
				V[i] = new Vector3(5.0f, 2.0f, 0.0f);
			}
		}

		if (!launched)
		{
			return;
		}

		float dt = 0.015f;

		//Step 1: run a simple particle system.
		for (int i = 0; i < V.Length; i++)
		{
			V[i] += gravity * dt;
			V[i] *= linear_decay;
		}

		//Step 2: Perform simple particle collision.
		Collision(1 / dt);

		// Step 3: Use shape matching to get new translation c and 
		// new rotation R. Update the mesh by c and R.
		//Shape Matching (translation)
		for (int i = 0; i < V.Length; i++)
		{
			Y[i] = X[i] + V[i] * dt;
		}

        // Compute c by averaging Y
        Vector3 c = new Vector3(0, 0, 0);
        for (int i = 0; i < Y.Length; ++i)
        {
            c += Y[i];
        }
        c /= Y.Length;

        // Compute matrix A
        Matrix4x4 A = Matrix4x4.zero;
        A.m33 = 1.0f;
        for (int i = 0; i < V.Length; i++)
        {
            // sum(yi - c) * ri^T  only 3by3 contains values
            Matrix4x4 t = vec3by1DotVec1by3(Y[i] - c, Q[i]);
            A[0, 0] += t[0, 0];
            A[0, 1] += t[0, 1];
            A[0, 2] += t[0, 2];
            A[1, 0] += t[1, 0];
            A[1, 1] += t[1, 1];
            A[1, 2] += t[1, 2];
            A[2, 0] += t[2, 0];
            A[2, 1] += t[2, 1];
            A[2, 2] += t[2, 2];
        }
        A = A * QQt.inverse;

        //Shape Matching (rotation)
        // compute R by polar decompose matrix A
        Matrix4x4 R = Get_Rotation(A);

		Update_Mesh(c, R, 1 / dt);
	}

    // ------------------------ helper functions ---------------------------------
    Matrix4x4 vec3by1DotVec1by3(Vector3 A, Vector3 B)
    {
        Matrix4x4 res = Matrix4x4.zero;
        res[3, 3] = 1.0f;

        res[0, 0] = A[0] * B[0];
        res[0, 1] = A[0] * B[1];
        res[0, 2] = A[0] * B[2];

        res[1, 0] = A[1] * B[0];
        res[1, 1] = A[1] * B[1];
        res[1, 2] = A[1] * B[2];

        res[2, 0] = A[2] * B[0];
        res[2, 1] = A[2] * B[1];
        res[2, 2] = A[2] * B[2];

        return res;
    }
}


