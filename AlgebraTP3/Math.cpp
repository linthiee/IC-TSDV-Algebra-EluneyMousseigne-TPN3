#include "Math.h"

#include "raymath.h"
#include <iostream>

void CalculateOBB(Model model, MyOBB& obb, Vector3 rotAxis, float rotAngle)
{
	Vector3 directions[3] = { { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

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

	obb.center = Vector3Scale(Vector3Add(min, max), 0.5f);
	obb.halfSizes = Vector3Scale(Vector3Subtract(max, min), 0.5f);

	Vector3 wereStartToRot = Vector3Zero();

	for (int i = 0; i < 3; i++)
	{
		wereStartToRot = Vector3CrossProduct(rotAxis, directions[i]);
		obb.localAxes[i] = directions[i] * cos(rotAngle * DEG2RAD) + wereStartToRot * sin(rotAngle * DEG2RAD) + rotAxis * Vector3DotProduct(rotAxis, directions[i]) * (1 - cos(rotAngle * DEG2RAD));
	}
}


void CalculateUniqueModelNormal(Model model, std::vector<Plane>& uniqueNormals)
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

		if (Vector3DotProduct(normal, toCenter) > 0)
		{
			normal = Vector3Negate(normal);
		}

		float d = -Vector3DotProduct(normal, v1);

		for (int j = 0; j < uniqueNormals.size(); j++)
		{
			if (Vector3DotProduct(normal, uniqueNormals[j].normal) > 0.99f &&
				fabsf(d - uniqueNormals[j].d) < 0.01f)
			{
				isUnique = false;
				break;
			}
		}

		if (isUnique)
		{
			uniqueNormals.push_back({ normal, d });
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

bool IsPointInsideMesh(Vector3 point, Model model, Matrix worldMatrix, std::vector<Plane>& uniqueNormals)
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

	point = Vector3Transform(point, MatrixInvert(worldMatrix));

	for (int i = 0; i < uniqueNormals.size(); i++)
	{
		if (Vector3DotProduct(uniqueNormals[i].normal, point) - uniqueNormals[i].d > EPSILON)
		{
			return false;
		}
	}

	return true;
}