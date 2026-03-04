#include "Math.h"

#include "raymath.h"
#include <iostream>

void CalculateOBB(Model model, MyOBB& obb, Matrix worldMatrix)
{
	Mesh mesh = model.meshes[0];

	obb.center = Vector3Zero();
	obb.halfSizes = Vector3Zero();

	Vector3 min = { mesh.vertices[0], mesh.vertices[1], mesh.vertices[2] };
	Vector3 max = min;

	for (int i = 1; i < mesh.vertexCount; i++)
	{
		Vector3 v1 = { mesh.vertices[i * 3 + 0], mesh.vertices[i * 3 + 1], mesh.vertices[i * 3 + 2] };

		if (v1.x < min.x)
		{
			min.x = v1.x;
		}
		if (v1.y < min.y)
		{
			min.y = v1.y;
		}
		if (v1.z < min.z)
		{
			min.z = v1.z;
		}

		if (v1.x > max.x)
		{
			max.x = v1.x;
		}
		if (v1.y > max.y)
		{
			max.y = v1.y;
		}
		if (v1.z > max.z)
		{
			max.z = v1.z;
		}
	}

	obb.center = Vector3Scale(Vector3Add(min, max), 0.5f); //get the center from the min and max of the obb (half of min + max)
	obb.halfSizes = Vector3Scale(Vector3Subtract(max, min), 0.5f); //get the halfsize of the obb

	obb.localAxes[0] = Vector3{
					worldMatrix.m0,
					worldMatrix.m1,
					worldMatrix.m2
	};

	obb.localAxes[1] = Vector3{
					worldMatrix.m4,
					worldMatrix.m5,
					worldMatrix.m6
	};

	obb.localAxes[2] = Vector3{
					worldMatrix.m8,
					worldMatrix.m9,
					worldMatrix.m10
	};
};


void UpdateOBB(MyOBB& obb, Matrix worldMatrix)
{
	obb.localAxes[0] = Vector3{
				worldMatrix.m0,
				worldMatrix.m1,
				worldMatrix.m2
	};

	obb.localAxes[1] = Vector3{
					worldMatrix.m4,
					worldMatrix.m5,
					worldMatrix.m6
	};

	obb.localAxes[2] = Vector3{
					worldMatrix.m8,
					worldMatrix.m9,
					worldMatrix.m10
	};
}

void DrawOBB(MyOBB obb)
{
	//DrawLine3D({ obb.center.x, (obb.center.y + obb.halfSizes.y) - obb.halfSizes.x, obb.center.z },
	//	{ obb.center.x + obb.halfSizes.x, (obb.center.y + obb.halfSizes.y), obb.center.z }, MAGENTA);

	Vector3 corner1 = { obb.center.x + obb.halfSizes.x,obb.center.y + obb.halfSizes.y ,obb.center.z + obb.halfSizes.z };
	Vector3 corner2 = { obb.center.x + obb.halfSizes.x,obb.center.y + obb.halfSizes.y ,obb.center.z - obb.halfSizes.z };
	Vector3 corner3 = { obb.center.x + obb.halfSizes.x,obb.center.y - obb.halfSizes.y ,obb.center.z - obb.halfSizes.z };
	Vector3 corner4 = { obb.center.x + obb.halfSizes.x,obb.center.y - obb.halfSizes.y ,obb.center.z + obb.halfSizes.z };
	Vector3 corner5 = { obb.center.x - obb.halfSizes.x,obb.center.y + obb.halfSizes.y ,obb.center.z + obb.halfSizes.z };
	Vector3 corner6 = { obb.center.x - obb.halfSizes.x,obb.center.y + obb.halfSizes.y ,obb.center.z - obb.halfSizes.z };
	Vector3 corner7 = { obb.center.x - obb.halfSizes.x,obb.center.y - obb.halfSizes.y ,obb.center.z - obb.halfSizes.z };
	Vector3 corner8 = { obb.center.x - obb.halfSizes.x,obb.center.y - obb.halfSizes.y ,obb.center.z + obb.halfSizes.z };
	

	DrawLine3D(corner1,corner2,BLUE);
	DrawLine3D(corner2,corner3, BLUE);
	DrawLine3D(corner3,corner4,BLUE);
	DrawLine3D(corner4,corner1,BLUE);

	DrawLine3D(corner5,corner6,BLUE);
	DrawLine3D(corner6,corner7,BLUE);
	DrawLine3D(corner7,corner8,BLUE);
	DrawLine3D(corner8,corner5,BLUE);

	DrawLine3D(corner1,corner5,BLUE);
	DrawLine3D(corner2,corner6,BLUE);
	DrawLine3D(corner3,corner7,BLUE);
	DrawLine3D(corner4,corner8,BLUE);

	DrawSphere(corner1,0.05f, MAGENTA);
	DrawSphere(corner2,0.05f, MAGENTA);
	DrawSphere(corner3,0.05f, MAGENTA);
	DrawSphere(corner4,0.05f, MAGENTA);
	DrawSphere(corner5, 0.05f, MAGENTA);
	DrawSphere(corner6, 0.05f, MAGENTA);
	DrawSphere(corner7, 0.05f, MAGENTA);
	DrawSphere(corner8, 0.05f, MAGENTA);
}

void CalculateUniqueModelPlanes(Model model, std::vector<Plane>& uniquePlanes) //calculate all the object normals outside of the update loop for better performance (and to save unecessary process)
{
	Mesh mesh = model.meshes[0];

	bool isUnique = true;

	for (int i = 0; i < mesh.triangleCount; i++)
	{
		isUnique = true;

		Vector3 v1 = { 0,0,0 };
		Vector3 v2 = { 0,0,0 };
		Vector3 v3 = { 0,0,0 };

		if (model.meshes[0].indices)
		{
			v1 = { mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 2] };
			v2 = { mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 2] };
			v3 = { mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 2] };
		}
		else
		{
			v1 = { mesh.vertices[(i * 3 + 0) * 3 + 0], mesh.vertices[(i * 3 + 0) * 3 + 1], mesh.vertices[(i * 3 + 0) * 3 + 2] };
			v2 = { mesh.vertices[(i * 3 + 1) * 3 + 0], mesh.vertices[(i * 3 + 1) * 3 + 1], mesh.vertices[(i * 3 + 1) * 3 + 2] };
			v3 = { mesh.vertices[(i * 3 + 2) * 3 + 0], mesh.vertices[(i * 3 + 2) * 3 + 1], mesh.vertices[(i * 3 + 2) * 3 + 2] };
		}

		Vector3 edge1 = Vector3Subtract(v2, v1);
		Vector3 edge2 = Vector3Subtract(v3, v1);
		Vector3 normal = Vector3Normalize(Vector3CrossProduct(edge1, edge2));

		Vector3 toCenter = Vector3Subtract(Vector3Zero(), v1);

		if (Vector3DotProduct(normal, toCenter) > 0) //if the normal points to the inside of the model negate it so it points to the outside
		{
			normal = Vector3Negate(normal);
		}

		float d = -Vector3DotProduct(normal, v1); //solve for d in the plane equation (normal * v1 + d = 0)

		for (int j = 0; j < uniquePlanes.size(); j++)
		{
			if (Vector3DotProduct(normal, uniquePlanes[j].normal) > 0.99f && //check if they point to the same direction
				fabsf(d - uniquePlanes[j].d) < 0.01f) //check if the distance from the origin is less than 0.1 units
			{
				isUnique = false;
				break;
			}
		}

		if (isUnique)
		{
			uniquePlanes.push_back({ normal, d });
		}
	}
}

MyAABB CalculateLocalAABB(Mesh mesh)
{
	MyAABB aabb;
	if (mesh.vertexCount == 0)
	{
		aabb.min = Vector3Zero();
		aabb.max = Vector3Zero();
		return aabb;
	}

	aabb.min = { mesh.vertices[0], mesh.vertices[1], mesh.vertices[2] };
	aabb.max = aabb.min;

	for (int i = 1; i < mesh.vertexCount; i++)
	{
		Vector3 v =
		{
			mesh.vertices[i * 3 + 0],
			mesh.vertices[i * 3 + 1],
			mesh.vertices[i * 3 + 2]
		};

		aabb.min.x = fminf(aabb.min.x, v.x);
		aabb.min.y = fminf(aabb.min.y, v.y);
		aabb.min.z = fminf(aabb.min.z, v.z);

		aabb.max.x = fmaxf(aabb.max.x, v.x);
		aabb.max.y = fmaxf(aabb.max.y, v.y);
		aabb.max.z = fmaxf(aabb.max.z, v.z);
	}
	return aabb;
}

MyAABB GetUpdatedAABB(MyAABB localBB, Matrix transform)
{
	Vector3 corners[8];
	corners[0] = { localBB.min.x, localBB.min.y, localBB.min.z };
	corners[1] = { localBB.max.x, localBB.min.y, localBB.min.z };
	corners[2] = { localBB.min.x, localBB.max.y, localBB.min.z };
	corners[3] = { localBB.min.x, localBB.min.y, localBB.max.z };
	corners[4] = { localBB.max.x, localBB.max.y, localBB.max.z };
	corners[5] = { localBB.min.x, localBB.max.y, localBB.max.z };
	corners[6] = { localBB.max.x, localBB.min.y, localBB.max.z };
	corners[7] = { localBB.max.x, localBB.max.y, localBB.min.z };

	for (int i = 0; i < 8; i++)
	{
		corners[i] = Vector3Transform(corners[i], transform);
	}

	Vector3 min = corners[0];
	Vector3 max = corners[0];
	for (int i = 1; i < 8; i++)
	{
		min.x = fminf(min.x, corners[i].x);
		min.y = fminf(min.y, corners[i].y);
		min.z = fminf(min.z, corners[i].z);

		max.x = fmaxf(max.x, corners[i].x);
		max.y = fmaxf(max.y, corners[i].y);
		max.z = fmaxf(max.z, corners[i].z);
	}

	return { min, max };
}

bool CheckCollisionAABB(MyAABB aabbA, MyAABB aabbB)
{
	if (aabbA.max.x < aabbB.min.x || aabbA.min.x > aabbB.max.x)
	{
		return false;
	}

	if (aabbA.max.y < aabbB.min.y || aabbA.min.y > aabbB.max.y)
	{
		return false;
	}

	if (aabbA.max.z < aabbB.min.z || aabbA.min.z > aabbB.max.z)
	{
		return false;
	}

	return true;
}

bool IsPointInsideMesh(Vector3 point, Model model, Matrix worldMatrix, std::vector<Plane>& uniquePlanes)
{
	if (model.meshCount == 0)
	{
		return false;
	}

	Mesh mesh = model.meshes[0];

	if (mesh.triangleCount == 0)
	{
		return false;
	}

	point = Vector3Transform(point, MatrixInvert(worldMatrix)); //convert the point into object coordinates for better performance

	for (int i = 0; i < uniquePlanes.size(); i++)
	{
		if (Vector3DotProduct(uniquePlanes[i].normal, point) + uniquePlanes[i].d > EPSILON)
		{
			return false;
		}
	}

	return true;
}

Vector3 operator+(Vector3& vector1, Vector3& vector2)
{
	return { vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z };
}

Vector3 operator-(Vector3& vector1, Vector3& vector2)
{
	return { vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z };
}

float getVectorMagnitude(Vector3 vector)
{
	return (sqrt((vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z))); //pythagoras theroem
}
