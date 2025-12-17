#include "Math.h"

#include "raymath.h"
#include <iostream>

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

bool IsPointInsideMesh(Vector3 point, Model model, Matrix worldMatrix)
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

    Vector3 modelCenter = Vector3Transform(Vector3Zero(), worldMatrix);

    Vector3 lastNormal = { 0,0,0 };

    for (int i = 0; i < mesh.triangleCount; i++)
    {
        Vector3 localV1 = { 0, 0, 0 };
        Vector3 localV2 = { 0, 0, 0 };
        Vector3 localV3 = { 0, 0, 0 };

        if (mesh.indices)
        {
            localV1 = { mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 0] * 3 + 2] };
            localV2 = { mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 1] * 3 + 2] };
            localV3 = { mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 0], mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 1], mesh.vertices[mesh.indices[i * 3 + 2] * 3 + 2] };
        }
        else
        {
            localV1 = { mesh.vertices[(i * 3 + 0) * 3 + 0], mesh.vertices[(i * 3 + 0) * 3 + 1], mesh.vertices[(i * 3 + 0) * 3 + 2] };
            localV2 = { mesh.vertices[(i * 3 + 1) * 3 + 0], mesh.vertices[(i * 3 + 1) * 3 + 1], mesh.vertices[(i * 3 + 1) * 3 + 2] };
            localV3 = { mesh.vertices[(i * 3 + 2) * 3 + 0], mesh.vertices[(i * 3 + 2) * 3 + 1], mesh.vertices[(i * 3 + 2) * 3 + 2] };
        }

        Vector3 v1 = Vector3Transform(localV1, worldMatrix);
        Vector3 v2 = Vector3Transform(localV2, worldMatrix);
        Vector3 v3 = Vector3Transform(localV3, worldMatrix);

        Vector3 edge1 = Vector3Subtract(v2, v1);
        Vector3 edge2 = Vector3Subtract(v3, v1);
        Vector3 normal = Vector3Normalize(Vector3CrossProduct(edge1, edge2));

        if (Vector3DotProduct(normal, lastNormal) > 0.99)
        {
            continue;
        }

        lastNormal = normal;

        Vector3 toCenter = Vector3Subtract(modelCenter, v1);

        if (Vector3DotProduct(normal, toCenter) > 0)
        {
            normal = Vector3Negate(normal);
        }

        Vector3 toPoint = Vector3Subtract(point, v1);
        float dot = Vector3DotProduct(toPoint, normal);

        if (dot > 0.0001f)
        {
            return false;
        }

        std::cout << mesh.triangleCount << "\r";
    }

    return true;
}