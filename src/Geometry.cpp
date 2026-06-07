/*
 * Copyright 2023 TU Wien, Institute of Visual Computing & Human-Centered Technology.
 * This file is part of the GCG Lab Framework and must not be redistributed.
 *
 * Original version created by Lukas Gersthofer and Bernhard Steiner.
 * Vulkan edition created by Johannes Unterguggenberger (junt@cg.tuwien.ac.at).
 */

#include "Geometry.h"
#include "PathUtils.h"
#include "Utils.h"
#include <glm/gtc/constants.hpp>

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#undef min
#undef max


constexpr float CORNELL_LEFT_R = 0.58f;
constexpr float CORNELL_LEFT_G = 0.57f;
constexpr float CORNELL_LEFT_B = 0.54f;
constexpr float CORNELL_RIGHT_R = 0.58f;
constexpr float CORNELL_RIGHT_G = 0.57f;
constexpr float CORNELL_RIGHT_B = 0.54f;

namespace {
struct ObjVertexKey {
	int positionIndex;
	int texcoordIndex;
	int normalIndex;

	bool operator==(const ObjVertexKey& other) const {
		return positionIndex == other.positionIndex
			&& texcoordIndex == other.texcoordIndex
			&& normalIndex == other.normalIndex;
	}
};

struct ObjVertexKeyHash {
	std::size_t operator()(const ObjVertexKey& key) const {
		std::size_t h = std::hash<int>{}(key.positionIndex);
		h ^= std::hash<int>{}(key.texcoordIndex) + 0x9e3779b9 + (h << 6) + (h >> 2);
		h ^= std::hash<int>{}(key.normalIndex) + 0x9e3779b9 + (h << 6) + (h >> 2);
		return h;
	}
};

int resolveObjIndex(int objIndex, int count) {
	if (objIndex > 0) {
		return objIndex - 1;
	}
	if (objIndex < 0) {
		return count + objIndex;
	}
	return -1;
}

std::array<int, 3> parseObjVertexToken(
	const std::string& token,
	int positionCount,
	int texcoordCount,
	int normalCount
) {
	std::array<int, 3> result{-1, -1, -1};
	std::stringstream ss(token);
	std::string indexString;
	int component = 0;

	while (std::getline(ss, indexString, '/') && component < 3) {
		if (!indexString.empty()) {
			const int parsed = std::stoi(indexString);
			if (component == 0) result[0] = resolveObjIndex(parsed, positionCount);
			if (component == 1) result[1] = resolveObjIndex(parsed, texcoordCount);
			if (component == 2) result[2] = resolveObjIndex(parsed, normalCount);
		}
		++component;
	}

	return result;
}
} // namespace

GeometryData loadObjGeometry(const std::string& model_file_path) {
	return loadObjGeometry(model_file_path, {});
}

GeometryData loadObjGeometry(const std::string& model_file_path, const std::vector<std::string>& include_object_names) {
	const std::string resolved_path = gcgFindFileInParentDir(model_file_path);
	if (resolved_path.empty()) {
		VKL_EXIT_WITH_ERROR("Could not find model file: " << model_file_path);
	}

	std::ifstream file(resolved_path);
	if (!file.good()) {
		VKL_EXIT_WITH_ERROR("Could not open model file: " << resolved_path);
	}

	std::vector<glm::vec3> obj_positions;
	std::vector<glm::vec3> obj_normals;
	std::vector<glm::vec2> obj_texcoords;

	GeometryData data;
	std::unordered_map<ObjVertexKey, uint32_t, ObjVertexKeyHash> unique_vertices;
	const std::unordered_set<std::string> include_objects_set(include_object_names.begin(), include_object_names.end());
	const bool include_all_objects = include_objects_set.empty();
	std::string current_object_name;

	std::string line;
	while (std::getline(file, line)) {
		if (line.empty() || line[0] == '#') {
			continue;
		}

		std::istringstream line_stream(line);
		std::string tag;
		line_stream >> tag;

		if (tag == "v") {
			glm::vec3 p(0.0f);
			line_stream >> p.x >> p.y >> p.z;
			obj_positions.push_back(p);
		}
		else if (tag == "o") {
			line_stream >> current_object_name;
		}
		else if (tag == "vn") {
			glm::vec3 n(0.0f);
			line_stream >> n.x >> n.y >> n.z;
			if (glm::length(n) > 0.0f) {
				n = glm::normalize(n);
			}
			obj_normals.push_back(n);
		}
		else if (tag == "vt") {
			glm::vec2 t(0.0f);
			line_stream >> t.x >> t.y;
			obj_texcoords.push_back(glm::vec2(t.x, 1.0f - t.y));
		}
		else if (tag == "f") {
			if (!include_all_objects) {
				if (current_object_name.empty() || include_objects_set.find(current_object_name) == include_objects_set.end()) {
					continue;
				}
			}

			std::vector<std::array<int, 3>> face_vertices;
			std::string vertex_token;
			while (line_stream >> vertex_token) {
				face_vertices.push_back(parseObjVertexToken(
					vertex_token,
					static_cast<int>(obj_positions.size()),
					static_cast<int>(obj_texcoords.size()),
					static_cast<int>(obj_normals.size())
				));
			}

			if (face_vertices.size() < 3) {
				continue;
			}

			for (size_t i = 1; i + 1 < face_vertices.size(); ++i) {
				const std::array<std::array<int, 3>, 3> tri = {
					face_vertices[0],
					face_vertices[i],
					face_vertices[i + 1]
				};

				for (const auto& v : tri) {
					if (v[0] < 0 || v[0] >= static_cast<int>(obj_positions.size())) {
						VKL_EXIT_WITH_ERROR("OBJ face uses an invalid position index in: " << resolved_path);
					}

					ObjVertexKey key{v[0], v[1], v[2]};
					auto it = unique_vertices.find(key);
					if (it == unique_vertices.end()) {
						const uint32_t new_index = static_cast<uint32_t>(data.positions.size());
						unique_vertices.emplace(key, new_index);

						data.positions.push_back(obj_positions[v[0]]);
						data.textureCoordinates.push_back(
							(v[1] >= 0 && v[1] < static_cast<int>(obj_texcoords.size()))
								? obj_texcoords[v[1]]
								: glm::vec2(0.0f)
						);
						data.normals.push_back(
							(v[2] >= 0 && v[2] < static_cast<int>(obj_normals.size()))
								? obj_normals[v[2]]
								: glm::vec3(0.0f)
						);

						data.indices.push_back(new_index);
					}
					else {
						data.indices.push_back(it->second);
					}
				}
			}
		}
	}

	if (data.positions.empty() || data.indices.empty()) {
		VKL_EXIT_WITH_ERROR("OBJ loader produced empty geometry for file: " << resolved_path);
	}

	bool has_non_zero_normals = false;
	for (const auto& n : data.normals) {
		if (glm::dot(n, n) > 0.0f) {
			has_non_zero_normals = true;
			break;
		}
	}

	if (!has_non_zero_normals) {
		data.normals.assign(data.positions.size(), glm::vec3(0.0f));
		for (size_t i = 0; i + 2 < data.indices.size(); i += 3) {
			const uint32_t i0 = data.indices[i];
			const uint32_t i1 = data.indices[i + 1];
			const uint32_t i2 = data.indices[i + 2];

			const glm::vec3 e1 = data.positions[i1] - data.positions[i0];
			const glm::vec3 e2 = data.positions[i2] - data.positions[i0];
			const glm::vec3 face_normal = glm::cross(e1, e2);

			data.normals[i0] += face_normal;
			data.normals[i1] += face_normal;
			data.normals[i2] += face_normal;
		}

		for (auto& n : data.normals) {
			if (glm::dot(n, n) > 0.0f) {
				n = glm::normalize(n);
			}
			else {
				n = glm::vec3(0.0f, 1.0f, 0.0f);
			}
		}
	}

	return data;
}

GeometryData createBoxGeometry(float width, float height, float depth)
{
	GeometryData data;

	data.positions = {
		// front
		glm::vec3(-width / 2.0f, -height / 2.0f,  depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f,  depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f,  depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f,  depth / 2.0f),
		// back
		glm::vec3(width / 2.0f, -height / 2.0f,  -depth / 2.0f),
		glm::vec3(-width / 2.0f, -height / 2.0f,  -depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f,  -depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f,  -depth / 2.0f),
		// right
		glm::vec3(width / 2.0f, -height / 2.0f,  depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f,  -depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f,  -depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f,  depth / 2.0f),
		// left
		glm::vec3(-width / 2.0f, -height / 2.0f,  -depth / 2.0f),
		glm::vec3(-width / 2.0f, -height / 2.0f,  depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f,  depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f,  -depth / 2.0f),
		// top
		glm::vec3(-width / 2.0f, height / 2.0f,  -depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f,  depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f,  depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f,  -depth / 2.0f),
		// bottom
		glm::vec3(-width / 2.0f, -height / 2.0f,  -depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f,  -depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f,  depth / 2.0f),
		glm::vec3(-width / 2.0f, -height / 2.0f,  depth / 2.0f)
	};

	data.normals = {
		// front
		glm::vec3(0, 0, 1),
		glm::vec3(0, 0, 1),
		glm::vec3(0, 0, 1),
		glm::vec3(0, 0, 1),
		// back
		glm::vec3(0, 0, -1),
		glm::vec3(0, 0, -1),
		glm::vec3(0, 0, -1),
		glm::vec3(0, 0, -1),
		// right
		glm::vec3(1, 0, 0),
		glm::vec3(1, 0, 0),
		glm::vec3(1, 0, 0),
		glm::vec3(1, 0, 0),
		// left
		glm::vec3(-1, 0, 0),
		glm::vec3(-1, 0, 0),
		glm::vec3(-1, 0, 0),
		glm::vec3(-1, 0, 0),
		// top
		glm::vec3(0, 1, 0),
		glm::vec3(0, 1, 0),
		glm::vec3(0, 1, 0),
		glm::vec3(0, 1, 0),
		// bottom
		glm::vec3(0, -1, 0),
		glm::vec3(0, -1, 0),
		glm::vec3(0, -1, 0),
		glm::vec3(0, -1, 0)
	};

	data.textureCoordinates = {
		// front
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1),
		// back
		glm::vec2(1, 1),
		glm::vec2(0, 1),
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		// right
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1),
		// left
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1),
		// top
		glm::vec2(0, 1),
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		// bottom
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1)
	};

	data.indices = {
		// front
		0, 1, 2,
		2, 3, 0,
		// back
		4, 5, 6,
		6, 7, 4,
		// right
		8, 9, 10,
		10, 11, 8,
		// left
		12, 13, 14,
		14, 15, 12,
		// top
		16, 17, 18,
		18, 19, 16,
		// bottom
		20, 21, 22,
		22, 23, 20
	};
	return data;
}


GeometryData createCornellBoxGeometry(float width, float height, float depth)
{
	GeometryData data;

	data.positions = {
		// back
		glm::vec3(width / 2.0f, -height / 2.0f, -depth / 2.0f),
		glm::vec3(-width / 2.0f, -height / 2.0f, -depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f, -depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f, -depth / 2.0f),
		// front
		glm::vec3(-width / 2.0f, -height / 2.0f, depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f, depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f, depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f, depth / 2.0f),
		// right
		glm::vec3(width / 2.0f, -height / 2.0f, depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f, -depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f, -depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f, depth / 2.0f),
		// left
		glm::vec3(-width / 2.0f, -height / 2.0f, -depth / 2.0f),
		glm::vec3(-width / 2.0f, -height / 2.0f, depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f, depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f, -depth / 2.0f),
		// top
		glm::vec3(-width / 2.0f, height / 2.0f, -depth / 2.0f),
		glm::vec3(-width / 2.0f, height / 2.0f, depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f, depth / 2.0f),
		glm::vec3(width / 2.0f, height / 2.0f, -depth / 2.0f),
		// bottom
		glm::vec3(-width / 2.0f, -height / 2.0f, -depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f, -depth / 2.0f),
		glm::vec3(width / 2.0f, -height / 2.0f, depth / 2.0f),
		glm::vec3(-width / 2.0f, -height / 2.0f, depth / 2.0f)
	};

	data.normals = {
		// back
		glm::vec3(0, 0, 1),
		glm::vec3(0, 0, 1),
		glm::vec3(0, 0, 1),
		glm::vec3(0, 0, 1),
		// front
		glm::vec3(0, 0, -1),
		glm::vec3(0, 0, -1),
		glm::vec3(0, 0, -1),
		glm::vec3(0, 0, -1),
		// right
		glm::vec3(-1, 0, 0),
		glm::vec3(-1, 0, 0),
		glm::vec3(-1, 0, 0),
		glm::vec3(-1, 0, 0),
		// left
		glm::vec3(1, 0, 0),
		glm::vec3(1, 0, 0),
		glm::vec3(1, 0, 0),
		glm::vec3(1, 0, 0),
		// top
		glm::vec3(0, -1, 0),
		glm::vec3(0, -1, 0),
		glm::vec3(0, -1, 0),
		glm::vec3(0, -1, 0),
		// bottom
		glm::vec3(0, 1, 0),
		glm::vec3(0, 1, 0),
		glm::vec3(0, 1, 0),
		glm::vec3(0, 1, 0)
	};

	glm::vec3 colors[6] = {
		glm::vec3(CORNELL_LEFT_R, CORNELL_LEFT_G, CORNELL_LEFT_B),    // left
		glm::vec3(CORNELL_RIGHT_R, CORNELL_RIGHT_G, CORNELL_RIGHT_B),    // right
		glm::vec3(0.96, 0.93, 0.85), // top
		glm::vec3(0.64, 0.64, 0.64), // bottom
		glm::vec3(0.76, 0.74, 0.68), // back
		glm::vec3(0.76, 0.74, 0.68)  // front (same as back)
	};

	data.colors = {
			colors[4],
			colors[4],
			colors[4],
			colors[4],

			colors[5],
			colors[5],
			colors[5],
			colors[5],

			colors[1],
			colors[1],
			colors[1],
			colors[1],

			colors[0],
			colors[0],
			colors[0],
			colors[0],

			colors[2],
			colors[2],
			colors[2],
			colors[2],

			colors[3],
			colors[3],
			colors[3],
			colors[3]
	};

	data.textureCoordinates = {
		// back
		glm::vec2(1, 1),
		glm::vec2(0, 1),
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		// front
		glm::vec2(0, 1),
		glm::vec2(1, 1),
		glm::vec2(1, 0),
		glm::vec2(0, 0),
		// right
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1),
		// left
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1),
		// top
		glm::vec2(0, 1),
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		// bottom
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1)
	};

	data.indices = {
		// back
		2, 1, 0,
		0, 3, 2,
		// front
		6, 5, 4,
		4, 7, 6,
		// right
		10, 9, 8,
		8, 11, 10,
		// left
		14, 13, 12,
		12, 15, 14,
		// top
		18, 17, 16,
		16, 19, 18,
		// bottom
		22, 21, 20,
		20, 23, 22
	};

	return data;
}
// clang-format on
GeometryData createCylinderGeometry(uint32_t segments, float height, float radius) {
    GeometryData data;


    // center vertices
    data.positions.push_back(glm::vec3(0, -height / 2.0f, 0));
    data.normals.push_back(glm::vec3(0, -1, 0));
    data.textureCoordinates.push_back(glm::vec2(0.5f, 0.5f));
    data.positions.push_back(glm::vec3(0, height / 2.0f, 0));
    data.normals.push_back(glm::vec3(0, 1, 0));
    data.textureCoordinates.push_back(glm::vec2(0.5f, 0.5f));

    // circle segments
    float angle_step = 2.0f * glm::pi<float>() / float(segments);
    for (uint32_t i = 0; i < segments; i++) {
        glm::vec3 circlePos = glm::vec3(glm::cos(i * angle_step) * radius, -height / 2.0f, glm::sin(i * angle_step) * radius);

        glm::vec2 squareToCircleUV = glm::vec2((circlePos.x / radius) * 0.5f + 0.5f, (circlePos.z / radius) * 0.5f + 0.5f);

        // bottom ring vertex
        data.positions.push_back(circlePos);
        data.positions.push_back(circlePos);
        data.normals.push_back(glm::vec3(0, -1, 0));
        data.normals.push_back(glm::normalize(circlePos - glm::vec3(0, -height / 2.0f, 0)));
        data.textureCoordinates.push_back(glm::vec2(squareToCircleUV.x, 1.0f - squareToCircleUV.y));
        data.textureCoordinates.push_back(glm::vec2(i * angle_step / (2.0f * glm::pi<float>()), 0));

        // top ring vertex
        circlePos.y = height / 2.0f;
        data.positions.push_back(circlePos);
        data.positions.push_back(circlePos);
        data.normals.push_back(glm::vec3(0, 1, 0));
        data.normals.push_back(glm::normalize(circlePos - glm::vec3(0, height / 2.0f, 0)));
        data.textureCoordinates.push_back(squareToCircleUV);
        data.textureCoordinates.push_back(glm::vec2(i * angle_step / (2.0f * glm::pi<float>()), 1));

        // bottom face
        data.indices.push_back(0);
        data.indices.push_back(2 + i * 4);
        data.indices.push_back(i == segments - 1 ? 2 : 2 + (i + 1) * 4);

        // top face
        data.indices.push_back(1);
        data.indices.push_back(i == segments - 1 ? 4 : (i + 2) * 4);
        data.indices.push_back((i + 1) * 4);

        // side faces
        data.indices.push_back(3 + i * 4);
        data.indices.push_back(i == segments - 1 ? 5 : 5 + (i + 1) * 4);
        data.indices.push_back(i == segments - 1 ? 3 : 3 + (i + 1) * 4);

        data.indices.push_back(i == segments - 1 ? 5 : 5 + (i + 1) * 4);
        data.indices.push_back(3 + i * 4);
        data.indices.push_back(5 + i * 4);
    }


    return data;
}

// Function to calculate binomial coefficient (n choose k)
int binomialCoefficient(int n, int k) {
    int result = 1;
    for (int i = 1; i <= k; ++i) {
        result *= (n - i + 1);
        result /= i;
    }
    return result;
}

// Function to calculate a point on the Bezier curve
glm::vec3 calculateBezierPoint(const std::vector<glm::vec3>& controlPoints, float t) {
    int n = controlPoints.size() - 1;
    glm::vec3 point(0.0f, 0.0f, 0.0f);
    for (int i = 0; i <= n; ++i) {
        float blend = binomialCoefficient(n, i) * pow(t, i) * pow(1 - t, n - i);
        point += controlPoints[i] * blend;
    }
    return point;
}

// Function to calculate derivative (tangent) at a point on the Bezier curve
glm::vec3 calculateBezierTangent(const std::vector<glm::vec3>& controlPoints, float t) {
    int n = controlPoints.size() - 1;
    glm::vec3 tangent(0.0f);

    for (int i = 0; i < n; ++i) {
        glm::vec3 diff = controlPoints[i + 1] - controlPoints[i];
        float blend = binomialCoefficient(n - 1, i) * pow(t, i) * pow(1 - t, (n - 1) - i);
        tangent += diff * (float)n * blend;
    }

    return glm::normalize(tangent);
}

// Function to generate a Bezier curve and subdivide it into N segments
void generateBezierCurve(const std::vector<glm::vec3>& controlPoints, int numSegments, std::vector<glm::vec3>& positions, std::vector<glm::vec3>& tangents) {
    float deltaT = 1.0f / (numSegments);
    for (int i = 0; i <= numSegments; ++i) {
        float t = i * deltaT;
        glm::vec3 position = calculateBezierPoint(controlPoints, t);
        glm::vec3 tangent = calculateBezierTangent(controlPoints, t);
        positions.push_back(position);
        tangents.push_back(tangent);
    }
}

GeometryData createBezierCylinderGeometry(uint32_t n_circular_segments, std::vector<glm::vec3> controlPoints, uint32_t s_bezier_segments, float radius) {
    GeometryData data;
    std::vector<glm::vec3> bezierPoints;
    std::vector<glm::vec3> bezierTangents;
    generateBezierCurve(controlPoints, s_bezier_segments, bezierPoints, bezierTangents);
    float v = 0;
    float angleStep = 2.0f * glm::pi<float>() / float(n_circular_segments);
    for (int point = 0; point < bezierPoints.size(); point++) {
        glm::vec3 forwardAxis = bezierTangents[point];
        glm::vec3 rightAxis = glm::normalize(glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), forwardAxis));
        glm::vec3 upAxis = glm::normalize(glm::cross(forwardAxis, rightAxis));

        // Circle segments
        uint32_t startIndex = data.positions.size();
        for (uint32_t i = 0; i < n_circular_segments; i++) {
            float cosTheta = glm::cos(i * angleStep);
            float sinTheta = glm::sin(i * angleStep);
            glm::vec3 circlePos = bezierPoints[point] + cosTheta * radius * rightAxis + sinTheta * radius * upAxis;
            data.positions.push_back(circlePos);

            data.normals.push_back(circlePos - bezierPoints[point]);
            float u = static_cast<float>(i) / static_cast<float>(n_circular_segments);
            data.textureCoordinates.push_back(glm::vec2(u, v));
            // Side faces
            if (point < bezierPoints.size() - 1) {
                data.indices.push_back(startIndex + i);
                data.indices.push_back(startIndex + (i + 1) % n_circular_segments);
                data.indices.push_back(startIndex + n_circular_segments + (i + 1) % n_circular_segments);

                data.indices.push_back(startIndex + n_circular_segments + (i + 1) % n_circular_segments);
                data.indices.push_back(startIndex + n_circular_segments + i);
                data.indices.push_back(startIndex + i % n_circular_segments);
            }
        }
        if (point < bezierPoints.size() - 1) {
            v += glm::min(glm::length(bezierPoints[point + 1] - bezierPoints[point]), 1.0f);
        }
           
    }
    // top face
    data.positions.push_back(bezierPoints[bezierPoints.size() - 1]);
    data.normals.push_back(bezierTangents[bezierTangents.size() - 1]);
    data.textureCoordinates.push_back(glm::vec2(0.5f, 0.5f));
    glm::vec3 forwardAxis = bezierTangents[bezierTangents.size() - 1];
    glm::vec3 rightAxis = glm::normalize(glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), forwardAxis));
    glm::vec3 upAxis = glm::normalize(glm::cross(forwardAxis, rightAxis));
    int numberpositions = data.positions.size() - 1;
    for (unsigned int i = 0; i <= n_circular_segments; i++) {
        data.normals.push_back(bezierTangents[bezierTangents.size() - 1]);
        glm::vec3 circlePosFlat = glm::vec3(glm::cos(i * angleStep) * radius, 0, glm::sin(i * angleStep) * radius);
        glm::vec2 squareToCircleUV = glm::vec2((circlePosFlat.x / radius) * 0.5f + 0.5f, (circlePosFlat.z / radius) * 0.5f + 0.5f);
        data.textureCoordinates.push_back(squareToCircleUV);
        float cosTheta = glm::cos(i * angleStep);
        float sinTheta = glm::sin(i * angleStep);

        glm::vec3 circlePos = bezierPoints[bezierPoints.size() - 1] + cosTheta * radius * rightAxis + sinTheta * radius * upAxis;
        data.positions.push_back(circlePos);
        data.indices.push_back(numberpositions + (i + 1));
        data.indices.push_back(numberpositions);
        data.indices.push_back(numberpositions + i);
    }

    // Bottom face
    data.positions.push_back(bezierPoints[0]);
    numberpositions = data.positions.size() - 1;
    forwardAxis = bezierTangents[0];
    rightAxis = glm::normalize(glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), forwardAxis));
    upAxis = glm::normalize(glm::cross(forwardAxis, rightAxis));
    data.normals.push_back(-bezierTangents[0]);
    data.textureCoordinates.push_back(glm::vec2(0.5f, 0.5f));
    for (unsigned int i = 0; i <= n_circular_segments; i++) {
        data.normals.push_back(-bezierTangents[0]);
        glm::vec3 circlePosFlat = glm::vec3(glm::cos(i * angleStep) * radius, 0, glm::sin(i * angleStep) * radius);
        glm::vec2 squareToCircleUV = glm::vec2((circlePosFlat.x / radius) * 0.5f + 0.5f, (circlePosFlat.z / radius) * 0.5f + 0.5f);
        data.textureCoordinates.push_back(squareToCircleUV);
        float cosTheta = glm::cos(i * angleStep);
        float sinTheta = glm::sin(i * angleStep);

        glm::vec3 circlePos = bezierPoints[0] + cosTheta * radius * rightAxis + sinTheta * radius * upAxis;
        data.positions.push_back(circlePos);
        data.indices.push_back(numberpositions);
        data.indices.push_back(numberpositions + (i + 1));
        data.indices.push_back(numberpositions + i);
    }
    return data;
}

GeometryData createSphereGeometry(uint32_t longitude_segments, uint32_t latitude_segments, float radius) {
    GeometryData data;


    data.positions.push_back(glm::vec3(0.0f, radius, 0.0f));
    data.positions.push_back(glm::vec3(0.0f, -radius, 0.0f));
    data.normals.push_back(glm::vec3(0.0f, radius, 0.0f));
    data.normals.push_back(glm::vec3(0.0f, -radius, 0.0f));
    data.textureCoordinates.push_back(glm::vec2(0.0f, 0.0f));
    data.textureCoordinates.push_back(glm::vec2(0.0f, 1.0f));

    // first and last ring
    for (uint32_t j = 0; j < longitude_segments; j++) {
        data.indices.push_back(0);
        data.indices.push_back(j == longitude_segments - 1 ? 2 : (j + 3));
        data.indices.push_back(2 + j);

        data.indices.push_back(2 + (latitude_segments - 2) * longitude_segments + j);
        data.indices.push_back(
            j == longitude_segments - 1 ? 2 + (latitude_segments - 2) * longitude_segments
                                        : 2 + (latitude_segments - 2) * longitude_segments + j + 1
        );
        data.indices.push_back(1);
    }

    // vertices and rings
    for (uint32_t i = 1; i < latitude_segments; i++) {
        float verticalAngle = float(i) * glm::pi<float>() / float(latitude_segments);
        for (uint32_t j = 0; j < longitude_segments; j++) {
            float horizontalAngle = float(j) * 2.0f * glm::pi<float>() / float(longitude_segments);
            glm::vec3 position = glm::vec3(
                radius * glm::sin(verticalAngle) * glm::cos(horizontalAngle),
                radius * glm::cos(verticalAngle),
                radius * glm::sin(verticalAngle) * glm::sin(horizontalAngle)
            );
            data.positions.push_back(position);
            data.normals.push_back(glm::normalize(position));

            data.textureCoordinates.push_back(glm::vec2(horizontalAngle / (2.0f * glm::pi<float>()), verticalAngle / glm::pi<float>()));

            if (i == 1)
                continue;

            data.indices.push_back(2 + (i - 1) * longitude_segments + j);
            data.indices.push_back(j == longitude_segments - 1 ? 2 + (i - 2) * longitude_segments : 2 + (i - 2) * longitude_segments + j + 1);
            data.indices.push_back(j == longitude_segments - 1 ? 2 + (i - 1) * longitude_segments : 2 + (i - 1) * longitude_segments + j + 1);

            data.indices.push_back(j == longitude_segments - 1 ? 2 + (i - 2) * longitude_segments : 2 + (i - 2) * longitude_segments + j + 1);
            data.indices.push_back(2 + (i - 1) * longitude_segments + j);
            data.indices.push_back(2 + (i - 2) * longitude_segments + j);
        }
    }

    return data;
}

Geometry createAndUploadIntoGpuMemory(const GeometryData& geometry_data) {
    if (geometry_data.positions.empty()) {
        VKL_EXIT_WITH_ERROR("An empty GeometryData::positions vector has been passed to createAndUploadIntoGpuMemory(...)");
    }
    if (geometry_data.indices.empty()) {
        VKL_EXIT_WITH_ERROR("An empty GeometryData::indices vector has been passed to createAndUploadIntoGpuMemory(...)");
    }

    Geometry result;

    // Create vertex positions buffer and copy data into it:
    size_t positions_buffer_byte_size = geometry_data.positions.size() * sizeof(geometry_data.positions[0]);
    result.positionsBuffer = vklCreateHostCoherentBufferAndUploadData(
        static_cast<const void *>(geometry_data.positions.data()),
        positions_buffer_byte_size,
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
    );

    // Create vertex color buffer and copy data into it:
    result.colorsBuffer = VK_NULL_HANDLE;
    if (geometry_data.colors.size() > 0) {
        size_t colors_buffer_byte_size = geometry_data.colors.size() * sizeof(geometry_data.colors[0]);
        result.colorsBuffer = vklCreateHostCoherentBufferAndUploadData(
            geometry_data.colors.data(), colors_buffer_byte_size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    }

    // Create vertex normals buffer and copy data into it:
    size_t normals_buffer_byte_size = geometry_data.normals.size() * sizeof(geometry_data.normals[0]);
    result.normalsBuffer = vklCreateHostCoherentBufferAndUploadData(
        geometry_data.normals.data(), normals_buffer_byte_size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    // Create vertex texture coordinates buffer and copy data into it:
    size_t texture_coordinates_buffer_byte_size = geometry_data.textureCoordinates.size() * sizeof(geometry_data.textureCoordinates[0]);
    result.textureCoordinatesBuffer = vklCreateHostCoherentBufferAndUploadData(
        geometry_data.textureCoordinates.data(),
        static_cast<VkDeviceSize>(texture_coordinates_buffer_byte_size),
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
    );
    // Create indices buffer and copy data into it:
    size_t indices_buffer_byte_size = geometry_data.indices.size() * sizeof(geometry_data.indices[0]);
    result.indicesBuffer = vklCreateHostCoherentBufferAndUploadData(
        geometry_data.indices.data(),
        static_cast<VkDeviceSize>(indices_buffer_byte_size),
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT
    );
    // Also store the number of indices:
    result.numberOfIndices = static_cast<uint32_t>(geometry_data.indices.size());

    return result;
}

void destroyGeometryGpuMemory(const Geometry& geometry) {
    vklDestroyHostCoherentBufferAndItsBackingMemory(geometry.indicesBuffer);
    vklDestroyHostCoherentBufferAndItsBackingMemory(geometry.textureCoordinatesBuffer);
    vklDestroyHostCoherentBufferAndItsBackingMemory(geometry.normalsBuffer);
    if (geometry.colorsBuffer != VK_NULL_HANDLE) {
        vklDestroyHostCoherentBufferAndItsBackingMemory(geometry.colorsBuffer);
    }
    vklDestroyHostCoherentBufferAndItsBackingMemory(geometry.positionsBuffer);
}

