#include "svg.hpp"

#include <bit>
#include <cstdio>
#include <charconv>
#include <memory>
#include <string_view>
#include <unordered_map>

#include "cframe2d.hpp"
#include "ellipse.hpp"
#include "string_hash.hpp"
#include "visitor_base.hpp"

#include <glm/trigonometric.hpp>

using namespace ZN;

static bool is_space(char c) {
	return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static bool is_alpha(char c) {
	return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

namespace {

struct Node {
	std::string_view tag;
	std::string_view id;
	Node* parent;
	std::unique_ptr<Node> firstChild;
	Node* lastChild{};
	std::unique_ptr<Node> nextSibling;

	CFrame2D transform{1.f};

	uint32_t fillColor{};
	uint32_t strokeColor{};
	float strokeWidth{};
	float opacity{1.f};
	float fillOpacity{1.f};
	float strokeOpacity{1.f};

	bool hasFill{false};
	bool hasStroke{false};

	template <typename Functor>
	void for_each_child(Functor&& func) {
		auto* pChild = firstChild.get();

		while (pChild) {
			func(*pChild);
			pChild = pChild->nextSibling.get();
		}
	}
};

struct PathNode : public Node {
	Path path;
};

struct GroupNode : public Node {
};

struct UseNode : public Node {
	std::string_view ref;
};

struct CircleNode : public Node {
	float cx;
	float cy;
	float radius;
};

struct EllipseNode : public Node {
	float cx;
	float cy;
	float rx;
	float ry;
};

struct RectNode : public Node {
	float x;
	float y;
	float width;
	float height;
	float rx;
	float ry;
};

class SVGParser {
	public:
		explicit SVGParser(const char* str, size_t size)
				: m_curr(str)
				, m_end(str + size) {}

		bool parse(Scene& result);
	private:
		const char* m_curr;
		const char* m_end;
		bool m_error{false};

		std::unique_ptr<Node> m_root;
		std::vector<Node*> m_parentStack;
		std::unordered_map<std::string, Node*, StringHash, std::equal_to<>> m_nodeById;

		float m_width{};
		float m_height{};

		void parse_tag();
		void parse_svg_tag(std::string_view tag);
		void parse_path(std::string_view tag);
		void parse_group_tag(std::string_view tag);
		void parse_use_tag(std::string_view tag);
		void parse_circle_tag(std::string_view tag);
		void parse_ellipse_tag(std::string_view tag);
		void parse_rect_tag(std::string_view tag);
		void skip_whitespace();

		bool consume(std::string_view str);

		template <typename Functor>
		void parse_tag_properties(Functor&& func);

		void parse_path_data(std::string_view pathData, Path& path);
		bool parse_path_data_position(const char*& pCurr, const char* pEnd, float* pResult);
		template <typename T>
		bool parse_numeric_value(const char*& pCurr, const char* pEnd, T& result);

		void parse_transform(std::string_view, CFrame2D& result);

		void parse_node_style(Node& node, std::string_view style);
		void apply_node_style(Node& node, std::string_view k, std::string_view v);

		template <typename T>
		T& make_node(std::string_view tag);

		void recursive_walk_tree(Node& node, const CFrame2D& parentTransform, int depth, Scene& dstScene);
};

}

bool svg_parse(const char* str, size_t size, Scene& result) {
	SVGParser parser(str, size);
	return parser.parse(result);
}

void Path::push_move_to(float x, float y) {
	commands.emplace_back(static_cast<uint32_t>(PathCommand::MOVE_TO));
	commands.emplace_back(std::bit_cast<uint32_t>(x));
	commands.emplace_back(std::bit_cast<uint32_t>(y));
}

void Path::push_move_to(const glm::vec2& p) {
	push_move_to(p.x, p.y);
}

void Path::push_line_to(float x, float y) {
	commands.emplace_back(static_cast<uint32_t>(PathCommand::LINE_TO));
	commands.emplace_back(std::bit_cast<uint32_t>(x));
	commands.emplace_back(std::bit_cast<uint32_t>(y));
}

void Path::push_line_to(const glm::vec2& p) {
	push_line_to(p.x, p.y);
}

void Path::push_quad_bezier_to(float x1, float y1, float x, float y) {
	commands.emplace_back(static_cast<uint32_t>(PathCommand::QUAD_BEZIER_TO));
	commands.emplace_back(std::bit_cast<uint32_t>(x1));
	commands.emplace_back(std::bit_cast<uint32_t>(y1));
	commands.emplace_back(std::bit_cast<uint32_t>(x));
	commands.emplace_back(std::bit_cast<uint32_t>(y));
}

void Path::push_bezier_to(float x1, float y1, float x2, float y2, float x, float y) {
	commands.emplace_back(static_cast<uint32_t>(PathCommand::BEZIER_TO));
	commands.emplace_back(std::bit_cast<uint32_t>(x1));
	commands.emplace_back(std::bit_cast<uint32_t>(y1));
	commands.emplace_back(std::bit_cast<uint32_t>(x2));
	commands.emplace_back(std::bit_cast<uint32_t>(y2));
	commands.emplace_back(std::bit_cast<uint32_t>(x));
	commands.emplace_back(std::bit_cast<uint32_t>(y));
}

void Path::push_arc_to(const glm::vec2& p0, const glm::vec2& p1, float rx, float ry, float xAxisRotation,
		uint32_t largeArcFlag, uint32_t sweepFlag) {
	glm::vec2 majorAxis;
	glm::vec2 center;
	glm::vec2 angleBounds;
	float eccentricity;

	// https://www.w3.org/TR/SVG11/implnote.html#ArcImplementationNotes
	// F.6.6 Correction of out-of-range radii
	// If rx = 0 or ry = 0, then treat this as a straight line from (x1, y1) to (x2, y2)
	// and stop
	if (rx == 0.f || ry == 0.f) {
		push_arc_to({}, {}, {}, 0.f, p1);
	}
	else {
		ellipse_arc(p0, p1, rx, ry, glm::radians(xAxisRotation), largeArcFlag, sweepFlag, majorAxis,
				center, angleBounds, eccentricity);
		push_arc_to(majorAxis, center, angleBounds, eccentricity, p1);
	}
}

void Path::push_arc_to(const glm::vec2& majorAxis, const glm::vec2& center, const glm::vec2& angleBounds,
		float eccentricity, const glm::vec2& pos) {
	commands.emplace_back(static_cast<uint32_t>(PathCommand::ARC_TO));
	commands.emplace_back(std::bit_cast<uint32_t>(majorAxis.x));
	commands.emplace_back(std::bit_cast<uint32_t>(majorAxis.y));
	commands.emplace_back(std::bit_cast<uint32_t>(center.x));
	commands.emplace_back(std::bit_cast<uint32_t>(center.y));
	commands.emplace_back(std::bit_cast<uint32_t>(angleBounds.x));
	commands.emplace_back(std::bit_cast<uint32_t>(angleBounds.y));
	commands.emplace_back(std::bit_cast<uint32_t>(pos.x));
	commands.emplace_back(std::bit_cast<uint32_t>(pos.y));
	commands.emplace_back(std::bit_cast<uint32_t>(eccentricity));
}

void Path::push_close() {
	commands.emplace_back(static_cast<uint32_t>(PathCommand::CLOSE));
}

void Path::print() const {
	for_each_command(VisitorBase {
		[](const MoveToCommand& c) {
			printf("M(%g, %g)\n", c.pos.x, c.pos.y);
		},
		[](const LineToCommand& c) {
			printf("L(%g, %g)\n", c.pos.x, c.pos.y);
		},
		[](const QuadBezierToCommand& c) {
			printf("Q(%g, %g, %g, %g)\n", c.c0.x, c.c0.y, c.pos.x, c.pos.y);
		},
		[](const BezierToCommand& c) {
			printf("C(%g, %g, %g, %g, %g, %g)\n", c.c0.x, c.c0.y, c.c1.x, c.c1.y, c.pos.x, c.pos.y);
		},
		[](const ArcToCommand& c) {
			printf("A(%g,%g @ %g,%g %g-%g)\n", glm::length(c.majorAxis),
					glm::length(c.majorAxis) * c.eccentricity, c.center.x, c.center.y,
					glm::degrees(c.angleBounds.x), glm::degrees(c.angleBounds.y));
		},
		[](CloseCommand) {
			puts("Z");
		},
	});
}

void Scene::push_path_command(uint32_t fillColor, uint32_t strokeColor, float strokeWidth) {
	commands.emplace_back(static_cast<uint32_t>(SceneCmdType::PATH));
	commands.emplace_back(fillColor);
	commands.emplace_back(strokeColor);
	commands.emplace_back(std::bit_cast<uint32_t>(strokeWidth));
	commands.emplace_back(static_cast<uint32_t>(paths.size()));
	paths.emplace_back();
}

void Scene::push_ellipse_command(uint32_t fillColor, uint32_t strokeColor, float strokeWidth,
		const glm::vec2& center, const glm::vec2& majorAxis, float eccentricity) {
	commands.emplace_back(static_cast<uint32_t>(SceneCmdType::ELLIPSE));
	commands.emplace_back(fillColor);
	commands.emplace_back(strokeColor);
	commands.emplace_back(std::bit_cast<uint32_t>(strokeWidth));
	commands.emplace_back(std::bit_cast<uint32_t>(center.x));
	commands.emplace_back(std::bit_cast<uint32_t>(center.y));
	commands.emplace_back(std::bit_cast<uint32_t>(majorAxis.x));
	commands.emplace_back(std::bit_cast<uint32_t>(majorAxis.y));
	commands.emplace_back(std::bit_cast<uint32_t>(eccentricity));
}

bool SVGParser::parse(Scene& dstScene) {
	while (m_curr < m_end && !m_error) {
		switch (*(m_curr++)) {
			case '<':
				parse_tag();
				break;
			default:
				break;
		}
	}

	if (m_error) {
		puts("SVG parse errored");
		return false;
	}

	if (m_root) {
		recursive_walk_tree(*m_root, CFrame2D{1.f}, 0, dstScene);
	}

	return true;
}

static float cross(const glm::vec2& u, const glm::vec2& v) {
	return u.x * v.y - u.y * v.x;
}

static void push_transformed_arc(Path& dstPath, const CFrame2D& transform, glm::vec2 majorAxis,
		const glm::vec2& center, glm::vec2 angleBounds, float eccentricity, const glm::vec2& p1) {
	auto minorAxis = transform.get_rotation_matrix()
			* (glm::vec2(-majorAxis.y, majorAxis.x) * eccentricity);
	majorAxis = transform.get_rotation_matrix() * majorAxis;
	auto majorAxisLen = glm::length(majorAxis);
	auto minorAxisLen = glm::length(minorAxis);
	eccentricity = minorAxisLen / majorAxisLen;

	if (cross(majorAxis, minorAxis) < 0.f) {
		//eccentricity = -eccentricity;
		angleBounds = -angleBounds;
	}

	dstPath.push_arc_to(majorAxis, transform * center, angleBounds, eccentricity, transform * p1);
}

static void make_transformed_arc_to(Path& dstPath, const CFrame2D& transform, const glm::vec2& p0,
		const glm::vec2& p1, float rx, float ry, float xAxisRotation, uint32_t largeArcFlag,
		uint32_t sweepFlag) {
	glm::vec2 center, majorAxis, angleBounds;
	float eccentricity;

	ellipse_arc(p0, p1, rx, ry, xAxisRotation, largeArcFlag, sweepFlag, majorAxis, center, angleBounds,
			eccentricity);

	push_transformed_arc(dstPath, transform, majorAxis, center, angleBounds, eccentricity, p1);
}

void SVGParser::recursive_walk_tree(Node& node, const CFrame2D& parentTransform, int depth, Scene& dstScene) {
	auto transform = parentTransform * node.transform;
	auto scl = transform.get_scale();
	auto averageScale = (scl.x + scl.y) * 0.5f;

	float strokeWidth = node.hasStroke ? node.strokeWidth * averageScale : 0.f;

	if (node.tag.compare("path") == 0) {
		auto& srcPath = static_cast<PathNode&>(node).path;

		dstScene.push_path_command(node.fillColor, node.strokeColor, strokeWidth);
		auto& dstPath = dstScene.paths.back();

		srcPath.for_each_command(VisitorBase {
			[&](const MoveToCommand& c) {
				auto p = transform * c.pos;
				dstPath.push_move_to(p.x, p.y);
			},
			[&](const LineToCommand& c) {
				auto p = transform * c.pos;
				dstPath.push_line_to(p.x, p.y);
			},
			[&](const QuadBezierToCommand& c) {
				auto c0 = transform * c.c0;
				auto p = transform * c.pos;
				dstPath.push_quad_bezier_to(c0.x, c0.y, p.x, p.y);
			},
			[&](const BezierToCommand& c) {
				auto c0 = transform * c.c0;
				auto c1 = transform * c.c1;
				auto p = transform * c.pos;
				dstPath.push_bezier_to(c0.x, c0.y, c1.x, c1.y, p.x, p.y);
			},
			[&](const ArcToCommand& c) {
				if (c.eccentricity == 0.f) {
					dstPath.push_arc_to({}, {}, {}, 0.f, transform * c.pos);
				}
				else {
					push_transformed_arc(dstPath, transform, c.majorAxis, c.center, c.angleBounds,
							c.eccentricity, c.pos);
				}
			},
			[&](CloseCommand) {
				dstPath.push_close();
			},
		});
	}
	else if (node.tag.compare("circle") == 0) {
		auto& c = static_cast<CircleNode&>(node);
		auto pos = transform * glm::vec2(c.cx, c.cy);

		auto axisX = transform.get_rotation_matrix() * glm::vec2(c.radius, 0.f);
		auto axisY = transform.get_rotation_matrix() * glm::vec2(0.f, c.radius);
		auto rx = glm::length(axisX);
		auto ry = glm::length(axisY);

		if (ry > rx) {
			std::swap(rx, ry);
			std::swap(axisX, axisY);
		}

		dstScene.push_ellipse_command(node.fillColor, node.strokeColor, strokeWidth, pos, axisX, ry / rx);
	}
	else if (node.tag.compare("ellipse") == 0) {
		auto& e = static_cast<EllipseNode&>(node);
		auto pos = transform * glm::vec2(e.cx, e.cy);
		auto axisX = transform.get_rotation_matrix() * glm::vec2(e.rx, 0.f);
		auto axisY = transform.get_rotation_matrix() * glm::vec2(0.f, e.ry);
		auto rx = glm::length(axisX);
		auto ry = glm::length(axisY);

		if (ry > rx) {
			std::swap(rx, ry);
			std::swap(axisX, axisY);
		}

		dstScene.push_ellipse_command(node.fillColor, node.strokeColor, strokeWidth, pos, axisX, ry / rx);
	}
	else if (node.tag.compare("rect") == 0) {
		auto& rect = static_cast<RectNode&>(node);

		dstScene.push_path_command(node.fillColor, node.strokeColor, strokeWidth);
		auto& dstPath = dstScene.paths.back();

		if (rect.rx == 0.f || rect.ry == 0.f) {
			dstPath.push_move_to(transform * glm::vec2(rect.x, rect.y));
			dstPath.push_line_to(transform * glm::vec2(rect.x + rect.width, rect.y));
			dstPath.push_line_to(transform * glm::vec2(rect.x + rect.width, rect.y + rect.height));
			dstPath.push_line_to(transform * glm::vec2(rect.x, rect.y + rect.height));
			dstPath.push_close();
		}
		else {
			glm::vec2 points[] = {
				glm::vec2(rect.x, rect.y + rect.ry),
				glm::vec2(rect.x + rect.rx, rect.y),
				glm::vec2(rect.x + rect.width - rect.rx, rect.y),
				glm::vec2(rect.x + rect.width, rect.y + rect.ry),
				glm::vec2(rect.x + rect.width, rect.y + rect.height - rect.ry),
				glm::vec2(rect.x + rect.width - rect.rx, rect.y + rect.height),
				glm::vec2(rect.x + rect.rx, rect.y + rect.height),
				glm::vec2(rect.x, rect.y + rect.height - rect.ry),
			};

			dstPath.push_move_to(transform * points[0]);
			make_transformed_arc_to(dstPath, transform, points[0], points[1], rect.rx, rect.ry, 0.f, 0, 1);
			dstPath.push_line_to(transform * points[2]);
			make_transformed_arc_to(dstPath, transform, points[2], points[3], rect.rx, rect.ry, 0.f, 0, 1);
			dstPath.push_line_to(transform * points[4]);
			make_transformed_arc_to(dstPath, transform, points[4], points[5], rect.rx, rect.ry, 0.f, 0, 1);
			dstPath.push_line_to(transform * points[6]);
			make_transformed_arc_to(dstPath, transform, points[6], points[7], rect.rx, rect.ry, 0.f, 0, 1);

			dstPath.push_close();
		}
	}
	else if (node.tag.compare("use") == 0) {
		Node* pRef = &node;

		for (;;) {
			if (auto it = m_nodeById.find(static_cast<UseNode&>(*pRef).ref.substr(1)); it != m_nodeById.end()) {
				pRef = it->second;

				if (it->second->tag.compare("use") != 0) {
					break;
				}
				else {
					transform *= pRef->transform;
				}
			}
			else {
				pRef = nullptr;
				break;
			}
		}

		if (pRef && (pRef->tag.compare("path") == 0 || pRef->tag.compare("g") == 0
				|| pRef->tag.compare("circle") == 0 || pRef->tag.compare("ellipse") == 0)
				|| pRef->tag.compare("rect") == 0) {
			recursive_walk_tree(*pRef, transform, depth + 1, dstScene);
		}
	}

	node.for_each_child([&](auto& child) {
		if (child.tag.compare("svg") == 0 || child.tag.compare("g") == 0
				|| child.tag.compare("path") == 0 || child.tag.compare("use") == 0
				|| child.tag.compare("ellipse") == 0 || child.tag.compare("circle") == 0
				|| child.tag.compare("rect") == 0) {
			recursive_walk_tree(child, transform, depth + 1, dstScene);
		}
	});
}

void SVGParser::skip_whitespace() {
	while (m_curr < m_end && is_space(*m_curr)) {
		++m_curr;
	}
}

void SVGParser::parse_tag() {
	skip_whitespace();

	if (m_curr >= m_end) {
		m_error = true;
		return;
	}

	bool closingTag = false;

	if (*m_curr == '/') {
		closingTag = true;
		++m_curr;
	}
	else if (!is_alpha(*m_curr)) {
		++m_curr;
		return;
	}

	auto* tagStart = m_curr;

	for (;;) {
		if (m_curr >= m_end) {
			m_error = true;
			return;
		}
		else if (*m_curr == '>' || is_space(*m_curr)) {
			break;
		}

		++m_curr;
	}

	std::string_view tag(tagStart, m_curr);

	//printf("tag: '%.*s' %s\n", (int)tag.size(), tag.data(), closingTag ? "CLOSE" : "");

	bool skipToEnd = false;
	Node* pObject = nullptr;

	if (!closingTag) {
		if (tag.compare("svg") == 0) {
			parse_svg_tag(tag);
		}
		else if (tag.compare("path") == 0) {
			parse_path(tag);
		}
		else if (tag.compare("g") == 0) {
			parse_group_tag(tag);
		}
		else if (tag.compare("use") == 0) {
			parse_use_tag(tag);
		}
		else if (tag.compare("circle") == 0) {
			parse_circle_tag(tag);
		}
		else if (tag.compare("ellipse") == 0) {
			parse_ellipse_tag(tag);
		}
		else if (tag.compare("rect") == 0) {
			parse_rect_tag(tag);
		}
		else {
			pObject = &make_node<Node>(tag);
		}
	}
	else {
		m_parentStack.pop_back();
		skipToEnd = true;
	}

	if (skipToEnd) {
		for (;;) {
			if (m_curr >= m_end) {
				m_error = true;
				return;
			}
			else if (*m_curr == '/') {
				if (!consume("/>")) {
					return;
				}

				m_parentStack.pop_back();

				break;
			}
			else if (*m_curr == '>') {
				++m_curr;
				break;
			}

			++m_curr;
		}
	}
	else if (pObject) {
		parse_tag_properties([this, pObject](auto k, auto v) {
			//printf("Got full kv [%.*s] = '%.*s'\n", k.size(), k.data(), v.size(), v.data());
			if (k.compare("id") == 0) {
				pObject->id = v;
				m_nodeById.emplace(std::make_pair(std::string(v), pObject));
			}
		});
	}
}

void SVGParser::parse_svg_tag(std::string_view tag) {
	auto& node = make_node<Node>(tag);

	parse_tag_properties([&](auto k, auto v) {
		if (k.compare("width") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), m_width); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("height") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), m_height); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("id") == 0) {
			node.id = v;
			m_nodeById.emplace(std::make_pair(std::string(v), &node));
		}
	});
}

void SVGParser::parse_path(std::string_view tag) {
	auto& pathNode = make_node<PathNode>(tag);

	parse_tag_properties([&](auto k, auto v) {
		//printf("Got full kv [%.*s] = '%.*s'\n", k.size(), k.data(), v.size(), v.data());

		if (k.compare("d") == 0) {
			parse_path_data(v, pathNode.path);
		}
		else if (k.compare("transform") == 0) {
			parse_transform(v, pathNode.transform);
		}
		else if (k.compare("id") == 0) {
			pathNode.id = v;
			m_nodeById.emplace(std::make_pair(std::string(v), &pathNode));
		}
		else if (k.compare("style") == 0) {
			parse_node_style(pathNode, v);
		}
	});
}

void SVGParser::parse_group_tag(std::string_view tag) {
	auto& groupNode = make_node<GroupNode>(tag);

	parse_tag_properties([&](auto k, auto v) {
		if (k.compare("transform") == 0) {
			parse_transform(v, groupNode.transform);
		}
		else if (k.compare("id") == 0) {
			groupNode.id = v;
			m_nodeById.emplace(std::make_pair(std::string(v), &groupNode));
		}
	});
}

void SVGParser::parse_use_tag(std::string_view tag) {
	auto& node = make_node<UseNode>(tag);

	parse_tag_properties([&](auto k, auto v) {
		if (k.compare("transform") == 0) {
			parse_transform(v, node.transform);
		}
		else if (k.compare("id") == 0) {
			node.id = v;
			m_nodeById.emplace(std::make_pair(std::string(v), &node));
		}
		else if (k.compare("xlink:href") == 0) {
			node.ref = v;
		}
	});
}

void SVGParser::parse_circle_tag(std::string_view tag) {
	auto& node = make_node<CircleNode>(tag);

	parse_tag_properties([&](auto k, auto v) {
		if (k.compare("transform") == 0) {
			parse_transform(v, node.transform);
		}
		else if (k.compare("id") == 0) {
			node.id = v;
			m_nodeById.emplace(std::make_pair(std::string(v), &node));
		}
		else if (k.compare("cx") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.cx); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("cy") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.cy); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("r") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.radius);
					ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("style") == 0) {
			parse_node_style(node, v);
		}
	});
}

void SVGParser::parse_ellipse_tag(std::string_view tag) {
	auto& node = make_node<EllipseNode>(tag);

	parse_tag_properties([&](auto k, auto v) {
		if (k.compare("transform") == 0) {
			parse_transform(v, node.transform);
		}
		else if (k.compare("id") == 0) {
			node.id = v;
			m_nodeById.emplace(std::make_pair(std::string(v), &node));
		}
		else if (k.compare("cx") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.cx); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("cy") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.cy); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("rx") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.rx); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("ry") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.ry); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("style") == 0) {
			parse_node_style(node, v);
		}
	});
}

void SVGParser::parse_rect_tag(std::string_view tag) {
	auto& node = make_node<RectNode>(tag);

	parse_tag_properties([&](auto k, auto v) {
		if (k.compare("transform") == 0) {
			parse_transform(v, node.transform);
		}
		else if (k.compare("id") == 0) {
			node.id = v;
			m_nodeById.emplace(std::make_pair(std::string(v), &node));
		}
		else if (k.compare("x") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.x); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("y") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.y); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("width") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.width);
					ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("height") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.height);
					ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("rx") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.rx); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("ry") == 0) {
			if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.ry); ec != std::errc{}) {
				m_error = true;
			}
		}
		else if (k.compare("style") == 0) {
			parse_node_style(node, v);
		}
	});
}

void SVGParser::parse_transform(std::string_view str, CFrame2D& result) {
	auto* pCurr = str.data();
	auto* pEnd = str.data() + str.size();

	if (str.find("translate(") == 0) {
		pCurr += std::string_view("translate(").size();

		if (auto [ptr, ec] = std::from_chars(pCurr, pEnd, result[2][0]); ec == std::errc{}) {
			pCurr = ptr;
		}
		else {
			m_error = true;
			return;
		}

		if (pCurr >= pEnd || *pCurr != ',') {
			m_error = true;
			return;
		}

		++pCurr;

		if (auto [ptr, ec] = std::from_chars(pCurr, pEnd, result[2][1]); ec == std::errc{}) {
			pCurr = ptr;
		}
		else {
			m_error = true;
			return;
		}
	}
	else if (str.find("matrix(") == 0) {
		pCurr += std::string_view("matrix(").size();

		for (int x = 0; x < 3; ++x) {
			for (int y = 0; y < 2; ++y) {
				if (auto [ptr, ec] = std::from_chars(pCurr, pEnd, result[x][y]); ec == std::errc{}) {
					pCurr = ptr;

					if ((x != 2 || y != 1) && (pCurr >= pEnd || *pCurr != ',')) {
						m_error = true;
						return;
					}

					++pCurr;
				}
				else {
					m_error = true;
					return;
				}
			}
		}
	}
	else {
		m_error = true;
		printf("Unknown transform type: %.*s\n", (int)str.size(), str.data());
	}
}

void SVGParser::parse_node_style(Node& node, std::string_view style) {
	const char* p = style.data();
	const char* pEnd = style.data() + style.size();
	const char* keyStart = style.data();
	const char* keyEnd = nullptr;

	for (;;) {
		if (p == pEnd) {
			if (keyEnd) {
				apply_node_style(node, std::string_view(keyStart, keyEnd), std::string_view(keyEnd + 1, p));
			}

			break;
		}
		else if (*p == ':') {
			keyEnd = p;
		}
		else if (*p == ';') {
			if (keyEnd) {
				apply_node_style(node, std::string_view(keyStart, keyEnd), std::string_view(keyEnd + 1, p));
			}

			keyStart = p + 1;
		}

		++p;
	}

	auto fillOpacity = static_cast<uint32_t>(node.opacity * node.fillOpacity * 255.f) << 24;
	auto strokeOpacity = static_cast<uint32_t>(node.opacity * node.strokeOpacity * 255.f) << 24;

	node.fillColor = node.hasFill ? (node.fillColor | fillOpacity) : 0u;
	node.strokeColor = node.hasStroke ? (node.strokeColor | strokeOpacity) : 0u;
}

void SVGParser::apply_node_style(Node& node, std::string_view k, std::string_view v) {
	if (k.compare("fill") == 0) {
		if (v.starts_with('#')) {
			uint32_t fillColor{};
			if (auto [ptr, ec] = std::from_chars(v.data() + 1, v.data() + v.size(), fillColor, 16);
					ec == std::errc{}) {
				
				node.fillColor = std::byteswap(fillColor) >> 8;
				node.hasFill = true;
			}
			else {
				m_error = true;
			}
		}
	}
	else if (k.compare("stroke") == 0) {
		if (v.starts_with('#')) {
			uint32_t strokeColor{};
			if (auto [ptr, ec] = std::from_chars(v.data() + 1, v.data() + v.size(), strokeColor, 16);
					ec == std::errc{}) {
				
				node.strokeColor = std::byteswap(strokeColor) >> 8;
				node.hasStroke = true;
			}
			else {
				m_error = true;
			}
		}
	}
	else if (k.compare("stroke-width") == 0) {
		float strokeWidth{};
		if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), strokeWidth); ec == std::errc{}) {
			node.strokeWidth = strokeWidth;
		}
		else {
			m_error = true;
		}
	}
	else if (k.compare("opacity") == 0) {
		if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.opacity); ec != std::errc{}) {
			m_error = true;
		}
	}
	else if (k.compare("fill-opacity") == 0) {
		if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.fillOpacity);
				ec != std::errc{}) {
			m_error = true;
		}
	}
	else if (k.compare("stroke-opacity") == 0) {
		if (auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), node.strokeOpacity);
				ec != std::errc{}) {
			m_error = true;
		}
	}
}

bool SVGParser::consume(std::string_view str) {
	if (m_curr + str.size() > m_end && str.compare(m_curr) != 0) {
		m_error = true;
		return false;
	}

	m_curr += str.size();
	return true;
}

template <typename Functor>
void SVGParser::parse_tag_properties(Functor&& func) {
	const char* keyStart = nullptr;
	const char* valueStart = nullptr;

	for (;;) {
		if (m_curr >= m_end) {
			m_error = true;
			return;
		}
		else if (!valueStart && *m_curr == '/') {
			if (!consume("/>")) {
				return;
			}

			m_parentStack.pop_back();

			break;
		}
		else if (!valueStart && *m_curr == '>') {
			++m_curr;
			break;
		}
		else if (*m_curr == '=' && keyStart && !valueStart) {
			if (!consume("=\"")) {
				return;
			}

			valueStart = m_curr;
			--m_curr;
		}
		else if (*m_curr == '"' && valueStart) {
			func(std::string_view(keyStart, valueStart - 2), std::string_view(valueStart, m_curr));
			valueStart = nullptr;
			keyStart = nullptr;
		}
		else if (!is_space(*m_curr) && !keyStart) {
			keyStart = m_curr;
		}

		++m_curr;
	}
}

void SVGParser::parse_path_data(std::string_view pathData, Path& path) {
	const char* pCurr = pathData.data();
	const char* pEnd = pathData.data() + pathData.size();

	float values[6];
	uint32_t largeArcFlag, sweepFlag;

	bool absolute = false;
	float cursorX = 0.f;
	float cursorY = 0.f;

	while (pCurr < pEnd && !m_error) {
		char curr = *(pCurr++);

		switch (curr) {
			// https://svgwg.org/svg2-draft/paths.html#PathDataMovetoCommands
			case 'm':
			case 'M':
				absolute = curr == 'M';

				while (parse_path_data_position(pCurr, pEnd, values)) {
					if (absolute) {
						cursorX = values[0];
						cursorY = values[1];
					}
					else {
						cursorX += values[0];
						cursorY += values[1];
					}

					path.push_move_to(cursorX, cursorY);
				}
				break;
			// https://svgwg.org/svg2-draft/paths.html#PathDataCubicBezierCommands
			case 'c':
			case 'C':
				absolute = curr == 'C';

				for (;;) {
					if (!parse_path_data_position(pCurr, pEnd, values)) {
						break;
					}

					if (!parse_path_data_position(pCurr, pEnd, values + 2)) {
						m_error = true;
						return;
					}

					if (!parse_path_data_position(pCurr, pEnd, values + 4)) {
						m_error = true;
						return;
					}

					if (absolute) {
						path.push_bezier_to(values[0], values[1], values[2], values[3], values[4], values[5]);

						cursorX = values[4];
						cursorY = values[5];
					}
					else {
						path.push_bezier_to(cursorX + values[0], cursorY + values[1], cursorX + values[2],
								cursorY + values[3], cursorX + values[4], cursorY + values[5]);

						cursorX += values[4];
						cursorY += values[5];
					}
				}

				break;
			// https://svgwg.org/svg2-draft/paths.html#PathDataLinetoCommands
			case 'l':
			case 'L':
				absolute = curr == 'L';

				while (parse_path_data_position(pCurr, pEnd, values)) {
					if (absolute) {
						cursorX = values[0];
						cursorY = values[1];
					}
					else {
						cursorX += values[0];
						cursorY += values[1];
					}

					path.push_line_to(cursorX, cursorY);
				}
				break;
			// https://svgwg.org/svg2-draft/paths.html#PathDataQuadraticBezierCommands
			case 'q':
			case 'Q':
				absolute = curr == 'Q';

				for (;;) {
					if (!parse_path_data_position(pCurr, pEnd, values)) {
						break;
					}

					if (!parse_path_data_position(pCurr, pEnd, values + 2)) {
						m_error = true;
						return;
					}

					if (absolute) {
						path.push_quad_bezier_to(values[0], values[1], values[2], values[3]);

						cursorX = values[2];
						cursorY = values[3];
					}
					else {
						path.push_quad_bezier_to(cursorX + values[0], cursorY + values[1], cursorX + values[2],
								cursorY + values[3]);

						cursorX += values[2];
						cursorY += values[3];
					}
				}

				break;
			// https://svgwg.org/svg2-draft/paths.html#PathDataEllipticalArcCommands
			case 'a':
			case 'A':
				absolute = curr == 'A';

				for (;;) {
					if (!parse_path_data_position(pCurr, pEnd, values)) {
						break;
					}

					if (!parse_numeric_value(pCurr, pEnd, values[2])) {
						puts("error on angle");
						m_error = true;
						break;
					}

					if (!parse_numeric_value(pCurr, pEnd, largeArcFlag)) {
						puts("error on largeArcFlag");
						m_error = true;
						break;
					}

					if (pCurr >= pEnd) {
						puts("Error on flag splitter");
						m_error = true;
						break;
					}

					++pCurr;

					if (!parse_numeric_value(pCurr, pEnd, sweepFlag)) {
						puts("error on sweepFlag");
						m_error = true;
						break;
					}

					if (!parse_path_data_position(pCurr, pEnd, values + 3)) {
						puts("error on pos");
						m_error = true;
						break;
					}

					if (absolute) {
						path.push_arc_to(glm::vec2(cursorX, cursorY), glm::vec2(values[3], values[4]), 
								values[0], values[1], values[2], largeArcFlag, sweepFlag);

						cursorX = values[3];
						cursorY = values[4];
					}
					else {
						path.push_arc_to(glm::vec2(cursorX, cursorY),
								glm::vec2(cursorX + values[3], cursorY + values[4]),
								values[0], values[1], values[2], largeArcFlag, sweepFlag);

						cursorX += values[3];
						cursorY += values[4];
					}
				}
				break;
			// https://svgwg.org/svg2-draft/paths.html#PathDataClosePathCommand
			case 'z':
			case 'Z':
				path.push_close();
				break;
		}
	}
}

bool SVGParser::parse_path_data_position(const char*& pCurr, const char* pEnd, float* pResult) {
	for (;;) {
		if (pCurr >= pEnd) {
			return false;
		}
		else if (!is_space(*pCurr)) {
			break;
		}

		++pCurr;
	}

	if (is_alpha(*pCurr)) {
		return false;
	}

	if (auto [ptr, ec] = std::from_chars(pCurr, pEnd, pResult[0]); ec == std::errc{}) {
		pCurr = ptr;
	}
	else {
		puts("Failed to parse n0");
		m_error = true;
		return false;
	}

	if (pCurr + 1 >= pEnd || *pCurr != ',') {
		m_error = true;
		return false;
	}

	++pCurr;

	if (auto [ptr, ec] = std::from_chars(pCurr, pEnd, pResult[1]); ec == std::errc{}) {
		pCurr = ptr;
	}
	else {
		puts("Failed to parse n1");
		m_error = true;
		return false;
	}

	return true;
}

template <typename T>
bool SVGParser::parse_numeric_value(const char*& pCurr, const char* pEnd, T& result) {
	for (;;) {
		if (pCurr >= pEnd) {
			return false;
		}
		else if (!is_space(*pCurr)) {
			break;
		}

		++pCurr;
	}

	if (is_alpha(*pCurr)) {
		return false;
	}

	if (auto [ptr, ec] = std::from_chars(pCurr, pEnd, result); ec == std::errc{}) {
		pCurr = ptr;
	}
	else {
		puts("Failed to parse n0");
		m_error = true;
		return false;
	}

	return true;
}

template <typename T>
T& SVGParser::make_node(std::string_view tag) {
	auto* parent = m_parentStack.empty() ? nullptr : m_parentStack.back();
	auto node = std::make_unique<T>();
	auto* pResult = node.get();

	pResult->tag = tag;

	node->parent = parent;

	if (parent) {
		if (parent->lastChild) {
			parent->lastChild->nextSibling = std::move(node);
		}
		else {
			parent->firstChild = std::move(node);
		}

		parent->lastChild = pResult;
	}
	else if (m_root) {
		auto* lastSibling = m_root->nextSibling.get();

		while (lastSibling) {
			if (!lastSibling->nextSibling) {
				lastSibling->nextSibling = std::move(node);
				break;
			}

			lastSibling = lastSibling->nextSibling.get();
		}
	}
	else {
		m_root = std::move(node);
	}

	m_parentStack.emplace_back(pResult);

	return *pResult;
}
