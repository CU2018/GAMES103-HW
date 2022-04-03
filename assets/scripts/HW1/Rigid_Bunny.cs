using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	public bool disableSliding = true;  // a is 0, vN is 0, large friction

	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision

	Vector3 gravity = new Vector3(0, -9.8f, 0);  // gravity
	float mu_T = 0.5f;  

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	void Collision_Impulse(Vector3 P, Vector3 N)
    {
		Vector3 x = transform.position;
		Quaternion q = transform.rotation;
		Matrix4x4 R = Matrix4x4.Rotate(q); // get the Rotation matrix R
		Vector3[] vertices = GetComponent<MeshFilter>().mesh.vertices;

		Vector3 rSum = new Vector3(0, 0, 0);
		Vector3 Rri, xi, vi;
		float count = 0;
		float rest = restitution;

		// Collision Detection
		foreach (Vector3 ri in vertices)                 
		{
			Rri = R * ri;
			xi = x + Rri;
			vi = v + Vector3.Cross(w, Rri);   // xi = x + R*ri;
			// check collision with position and velocity 
			if (Vector3.Dot(xi - P, N) < 0 && Vector3.Dot(vi, N) < 0)
			{
				rSum += Rri;
				count += 1;
			}
		}

		// Collision Response
		if (count != 0)                                                         // Collision Response
		{
			Rri = rSum / count;
			vi = v + Vector3.Cross(w, Rri);
			Matrix4x4 RriCrossMat = Get_Cross_Matrix(Rri);

			// Compute vi_new
			float mag = vi.magnitude;
			if (mag < 0.3)
			{
				rest = 0;
			}
			Vector3 vN = Vector3.Dot(vi, N) * N;  // normal direction v
			Vector3 vT = vi - vN;   // tangenet direction v
			rest = Mathf.Max(rest - 0.0005f, 0);  // decrease the restituation to reduce oscillation
			float a = Mathf.Max(0, 1.0f - mu_T * (1.0f + rest)) * Vector3.Magnitude(vN) / Vector3.Magnitude(vT);
			Vector3 vi_new = -rest * vN + a * vT;

			// Compute impulse j
			Matrix4x4 I = R * I_ref * R.transpose;
			Matrix4x4 K = Matrix4x4.zero;
			// fill inverse of mass on the diagonal (1/M * 1)
			K.m00 = K.m11 = K.m22 = K.m33 = 1 / mass;
			Matrix4x4 kRest = RriCrossMat * I.inverse * RriCrossMat;
			// K = [1/M * 1] - Ri * I^-1 * Ri.T
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					K[i, j] -= kRest[i, j];
				}
			}
			
			Vector3 J;
			if (!disableSliding)  // with vN
				J = K.inverse * (vi_new - vi);
			else  // without vN
				J = K.inverse * (-rest * vN - vi);
			v += J / mass;
			Vector4 w1 = I.inverse * Vector3.Cross(Rri, J);
			w += new Vector3(w1.x, w1.y, w1.z);
		}
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		// Part I: Update velocities
		if (!launched)  // Disable the linear motion and the position update if not launched
			return;
		v += dt * gravity;
		v *= linear_decay;
		w *= angular_decay;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));  // handle floor collision
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));     // handle wall collision

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		x += v * dt;
		//Update angular status
		Quaternion q = transform.rotation;
        float halfDt = 0.5f * dt;
        Quaternion a = new Quaternion(halfDt * w.x, halfDt * w.y, halfDt * w.z, 0) * q;
        q = addTwoQuaternion(q, a).normalized;

        // Part IV: Assign to the object
        transform.position = x;
		transform.rotation = q;
	}

	// ------------------------ helper functions ---------------------------------
	private Quaternion addTwoQuaternion(Quaternion q1, Quaternion q2)
    {
		return new Quaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
    }
}
