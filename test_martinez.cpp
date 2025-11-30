#include <catch2/catch_test_macros.hpp>

#include "martinez_impl.hpp"

#include <simdjson.h>

#include <set>
#include <cfloat>

using namespace Martinez;

struct Feature {
	std::vector<Polygon> polygons;
};

static bool load_polygon(simdjson::ondemand::array& coords, Polygon& polygon, const char* fileName) {
	for (auto path : coords) {
		polygon.subShapes.emplace_back();
		auto& subshape = polygon.subShapes.back();

		simdjson::ondemand::array aryPath;
		if (path.get(aryPath) != 0) {
			printf("Failed to get path array: %s\n", fileName);
			return false;
		}

		for (auto point : aryPath) {
			simdjson::ondemand::array aryPoint;

			if (point.get(aryPoint) != 0) {
				printf("Failed to get point array: %s\n", fileName);
				return false;
			}

			real_t pointValues[2];
			size_t counter = 0;

			for (auto num : aryPoint) {
				if (num.get(pointValues[counter++]) != 0) {
					printf("Failed to get point value: %s\n", fileName);
					return false;
				}

				if (counter > 2) {
					break;
				}
			}

			subshape.points.emplace_back(pointValues[0], pointValues[1]);
		}
	}

	return true;
}

template <typename T>
static void load_feature(T& d, const char* fileName, std::vector<Feature>& features) {
	std::string_view type;
	if (d["geometry"]["type"].get(type) != 0) {
		printf("Failed to get geometry type: %s\n", fileName);
		return;
	}

	simdjson::ondemand::array coords;
	if (d["geometry"]["coordinates"].get(coords) != 0) {
		printf("Failed to get coordinates: %s\n", fileName);
		return;
	}

	Feature feature;

	if (type.compare("Polygon") == 0) {
		feature.polygons.emplace_back();
		if (!load_polygon(coords, feature.polygons.back(), fileName)) {
			return;
		}
	}
	else if (type.compare("MultiPolygon") == 0) {
		for (auto poly : coords) {
			simdjson::ondemand::array aryPoly;
			if (poly.get(aryPoly) != 0) {
				printf("Failed to get multipoly array: %s\n", fileName);
				return;
			}

			feature.polygons.emplace_back();
			if (!load_polygon(aryPoly, feature.polygons.back(), fileName)) {
				return;
			}
		}
	}
	else {
		printf("Invalid geometry type: %.*s: %s\n\n", (int)type.size(), type.data(), fileName);
		return;
	}

	features.emplace_back(std::move(feature));
}

static std::vector<Feature> load_json_file(const char* fileName) {
	auto text = simdjson::padded_string::load(fileName);
	simdjson::ondemand::parser parser;
	auto d = parser.iterate(text);

	std::string_view objectType;
	if (d["type"].get(objectType) != 0) {
		printf("Failed to get type!: %s\n", fileName);
		return {};
	}

	std::vector<Feature> features;

	if (objectType.compare("Feature") == 0) {
		load_feature(d, fileName, features);
	}
	else if (objectType.compare("FeatureCollection") == 0) {
		simdjson::ondemand::array aryFeatures;
		if (d["features"].get(aryFeatures) != 0) {
			printf("Failed to get features: %s\n", fileName);
			return {};
		}

		for (auto feature : aryFeatures) {
			load_feature(feature, fileName, features);
		}
	}
	else {
		printf("Invalid object type: %.*s\n", (int)objectType.size(), objectType.data());
		return {};
	}

	return features;
}

TEST_CASE("fill event queue", "martinez") {
	auto features = load_json_file("fixtures/two_triangles.geojson");
	REQUIRE(!features.empty());

	auto& subject = features[0].polygons[0];
	auto& clipping = features[1].polygons[0];

	AABB sbbox{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
	AABB cbbox{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};

	std::vector<std::unique_ptr<SweepEvent>> eventOwner;
	EventQueue queue;
	fill_queue(queue, eventOwner, subject, clipping, sbbox, cbbox, BooleanOperation::UNION);
	std::vector<SweepEvent*> sortedPoints;

	SECTION("bboxes") {
		REQUIRE(sbbox == AABB{20, -113.5f, 226.5f, 74});
		REQUIRE(cbbox == AABB{54.5f, -198, 239.5f, 33.5f});
	}

	glm::vec<2, real_t> points[] = {{20, -23.5f}, {20, -23.5f}, {54.5f, -170.5f}, {54.5f, -170.5f}, {140.5f, 33.5f},
		{140.5f, 33.5f}, {170.f, 74.f}, {170.f, 74.f}, {226.5f, -113.5f}, {226.5f, -113.5f}, {239.5f, -198},
		{239.5f, -198}};
	glm::vec<2, real_t> otherPoints[] = {{226.5f, -113.5f}, {170.f, 74.f}, {239.5f, -198}, {140.5f, 33.5f},
		{54.5f, -170.5f}, {239.5f, -198}, {20, -23.5f}, {226.5f, -113.5f}, {20, -23.5f}, {170, 74},
		{54.5f, -170.5f}, {140.5f, 33.5f}};

	bool lefts[] = {true, true, true, true, false, true, false, true, false, false, false, false};
	bool otherLefts[] = {false, false, false, false, true, false, true, false, true, true, true, true};

	for (size_t i = 0; i < 12; ++i) {
		auto* currentPoint = queue.top();
		queue.pop();

		sortedPoints.emplace_back(currentPoint);

		REQUIRE(currentPoint->point == points[i]);
		REQUIRE(currentPoint->left == lefts[i]);
		REQUIRE(currentPoint->otherEvent->point == otherPoints[i]);
		REQUIRE(currentPoint->otherEvent->left == otherLefts[i]);
	}

	for (size_t i = 0; i < sortedPoints.size() - 1; ++i) {
		REQUIRE(CompareEvents{}(sortedPoints[i], sortedPoints[i + 1]) == std::strong_ordering::less);
	}
}

TEST_CASE("queue", "martinez") {
	SECTION("queue should process least(by x) sweep event first") {
		auto e1 = std::make_unique<SweepEvent>(glm::vec<2, real_t>(0, 0));
		auto e2 = std::make_unique<SweepEvent>(glm::vec<2, real_t>(0.5f, 0.5f));
		EventQueue queue;

		queue.push(e1.get());
		queue.push(e2.get());

		REQUIRE(queue.top() == e1.get());
		queue.pop();
		REQUIRE(queue.top() == e2.get());
		queue.pop();
	}

	SECTION("queue should process least(by y) sweep event first") {
		auto e1 = std::make_unique<SweepEvent>(glm::vec<2, real_t>(0, 0));
		auto e2 = std::make_unique<SweepEvent>(glm::vec<2, real_t>(0, 0.5f));
		EventQueue queue;

		queue.push(e1.get());
		queue.push(e2.get());

		REQUIRE(queue.top() == e1.get());
		queue.pop();
		REQUIRE(queue.top() == e2.get());
		queue.pop();
	}

	SECTION("queue should process least(by left prop) sweep event first") {
		auto e1 = std::make_unique<SweepEvent>(glm::vec<2, real_t>(0, 0), nullptr, 0, EdgeType::NORMAL, true);
		auto e2 = std::make_unique<SweepEvent>(glm::vec<2, real_t>(0, 0), nullptr, 0, EdgeType::NORMAL, false);
		EventQueue queue;

		queue.push(e1.get());
		queue.push(e2.get());

		REQUIRE(queue.top() == e2.get());
		queue.pop();
		REQUIRE(queue.top() == e1.get());
		queue.pop();
	}

	SECTION("sweep event comparison x coordinates") {
		SweepEvent e1{{0, 0}};
		SweepEvent e2{{0.5f, 0.5f}};

		REQUIRE(CompareEvents{}(&e1, &e2) == std::strong_ordering::less);
		REQUIRE(CompareEvents{}(&e2, &e1) == std::strong_ordering::greater);
	}

	SECTION("sweep event comparison y coordinates") {
		SweepEvent e1{{0, 0}};
		SweepEvent e2{{0, 0.5f}};

		REQUIRE(CompareEvents{}(&e1, &e2) == std::strong_ordering::less);
		REQUIRE(CompareEvents{}(&e2, &e1) == std::strong_ordering::greater);
	}

	SECTION("sweep event comparison not left first") {
		SweepEvent e1{.point = {}, .left = true};
		SweepEvent e2{.point = {}, .left = false};

		REQUIRE(CompareEvents{}(&e1, &e2) == std::strong_ordering::greater);
		REQUIRE(CompareEvents{}(&e2, &e1) == std::strong_ordering::less);
	}

	SECTION("sweep event comparison shared start point not collinear edges") {
		SweepEvent e1b{.point = {1, 1}, .left = false};
		SweepEvent e1{.point = {}, .otherEvent = &e1b, .left = true};
		SweepEvent e2b{.point={2, 3}, .left = false};
		SweepEvent e2{.point = {}, .otherEvent = &e2b, .left = true};

		REQUIRE(CompareEvents{}(&e1, &e2) == std::strong_ordering::less);
		REQUIRE(CompareEvents{}(&e2, &e1) == std::strong_ordering::greater);
	}

	SECTION("sweep event comparison collinear edges") {
		SweepEvent e1b{.point = {1, 1}, .left = false};
		SweepEvent e1{.point = {}, .otherEvent = &e1b, .left = true, .isSubject = true};
		SweepEvent e2b{.point={2, 3}, .left = false};
		SweepEvent e2{.point = {}, .otherEvent = &e2b, .left = true, .isSubject = false};

		REQUIRE(CompareEvents{}(&e1, &e2) == std::strong_ordering::less);
		REQUIRE(CompareEvents{}(&e2, &e1) == std::strong_ordering::greater);
	}
}

TEST_CASE("compare segments not collinear", "martinez") {
	SECTION("shared left point - right point first") {
		std::multiset<SweepEvent*, CompareSegmentsLess> tree;
		SweepEvent se1b{.point={1, 1}, .left=false};
		SweepEvent se1{.point = {}, .otherEvent = &se1b, .left = true};
		SweepEvent se2b{.point={2, 3}, .left = false};
		SweepEvent se2{.point = {}, .otherEvent = &se2b, .left = true};

		tree.insert(&se1);
		tree.insert(&se2);

		auto maxNode = tree.end();
		--maxNode;

		REQUIRE((*maxNode)->otherEvent->point == glm::vec<2, real_t>{2, 3});
		REQUIRE((*tree.begin())->otherEvent->point == glm::vec<2, real_t>{1, 1});
	}

	SECTION("different left point - right point y coord to sort") {
		std::multiset<SweepEvent*, CompareSegmentsLess> tree;
		SweepEvent se1b{.point={1, 1}, .left=false};
		SweepEvent se1{.point = {0, 1}, .otherEvent = &se1b, .left = true};
		SweepEvent se2b{.point={2, 3}, .left = false};
		SweepEvent se2{.point = {0, 2}, .otherEvent = &se2b, .left = true};

		tree.insert(&se1);
		tree.insert(&se2);

		auto maxNode = tree.end();
		--maxNode;

		REQUIRE((*tree.begin())->otherEvent->point == glm::vec<2, real_t>{1, 1});
		REQUIRE((*maxNode)->otherEvent->point == glm::vec<2, real_t>{2, 3});
	}

	SECTION("events order in sweep line") {
		SweepEvent se1b{.point={2, 1}, .left=false};
		SweepEvent se1{.point = {0, 1}, .otherEvent = &se1b, .left = true};
		SweepEvent se2b{.point={2, 3}, .left = false};
		SweepEvent se2{.point = {-1, 0}, .otherEvent = &se2b, .left = true};

		SweepEvent se3b{.point={3, 4}, .left=false};
		SweepEvent se3{.point = {0, 1}, .otherEvent = &se3b, .left = true};
		SweepEvent se4b{.point={3, 1}, .left = false};
		SweepEvent se4{.point = {-1, 0}, .otherEvent = &se4b, .left = true};

		REQUIRE(CompareEvents{}(&se1, &se2) == std::strong_ordering::greater);
		REQUIRE(!se2.is_below(se1.point));
		REQUIRE(se2.is_above(se1.point));

		REQUIRE(CompareSegments{}(&se1, &se2) == std::strong_ordering::less);
		REQUIRE(CompareSegments{}(&se2, &se1) == std::strong_ordering::greater);

		REQUIRE(CompareEvents{}(&se3, &se4) == std::strong_ordering::greater);
		REQUIRE(!se4.is_above(se3.point));
	}

	SECTION("first point is below") {
		SweepEvent se2b{.point={2, 1}, .left=false};
		SweepEvent se2{.point = {0, 1}, .otherEvent = &se2b, .left = true};
		SweepEvent se1b{.point={2, 3}, .left = false};
		SweepEvent se1{.point = {-1, 0}, .otherEvent = &se1b, .left = true};

		REQUIRE(!se1.is_below(se2.point));
		REQUIRE(CompareSegments{}(&se1, &se2) == std::strong_ordering::greater);
	}
}

TEST_CASE("compare segments collinear", "martinez") {
	SECTION("collinear segments") {
		SweepEvent se1b{.point={5, 1}, .left=false};
		SweepEvent se1{.point = {1, 1}, .otherEvent = &se1b, .left = true, .isSubject = true};
		SweepEvent se2b{.point={3, 1}, .left = false};
		SweepEvent se2{.point = {2, 1}, .otherEvent = &se2b, .left = true, .isSubject = false};

		REQUIRE(se1.isSubject != se2.isSubject);
		REQUIRE(CompareSegments{}(&se1, &se2) == std::strong_ordering::less);
	}

	SECTION("collinear shared left point") {
		SweepEvent se1b{.point={5, 1}, .left=false};
		SweepEvent se1{.point = {0, 1}, .otherEvent = &se1b, .left = true, .isSubject = false};
		SweepEvent se2b{.point={3, 1}, .left = false};
		SweepEvent se2{.point = {0, 1}, .otherEvent = &se2b, .left = true, .isSubject = false};

		se1.contourID = 1;
		se2.contourID = 2;

		REQUIRE(se1.isSubject == se2.isSubject);
		REQUIRE(se1.point == se2.point);

		REQUIRE(CompareSegments{}(&se1, &se2) == std::strong_ordering::less);

		se1.contourID = 2;
		se2.contourID = 1;

		REQUIRE(CompareSegments{}(&se1, &se2) == std::strong_ordering::greater);
	}

	SECTION("collinear same polygon different left points") {
		SweepEvent se1b{.point={5, 1}, .left=false};
		SweepEvent se1{.point = {1, 1}, .otherEvent = &se1b, .left = true, .isSubject = true};
		SweepEvent se2b{.point={3, 1}, .left = false};
		SweepEvent se2{.point = {2, 1}, .otherEvent = &se2b, .left = true, .isSubject = true};

		REQUIRE(se1.isSubject == se2.isSubject);
		REQUIRE(se1.point != se2.point);
		REQUIRE(CompareSegments{}(&se1, &se2) == std::strong_ordering::less);
		REQUIRE(CompareSegments{}(&se2, &se1) == std::strong_ordering::greater);
	}
}

TEST_CASE("sweep event", "martinez") {
	SECTION("isBelow") {
		SweepEvent s1b{.point = {1, 1}};
		SweepEvent s1{.point = {}, .otherEvent = &s1b, .left = true};
		SweepEvent s2b{.point = {0, 0}};
		SweepEvent s2{.point = {0, 1}, .otherEvent = &s2b};

		REQUIRE(s1.is_below({0, 1}));
		REQUIRE(s1.is_below({1, 2}));
		REQUIRE(!s1.is_below({0, 0}));
		REQUIRE(!s1.is_below({5, -1}));

		REQUIRE(!s2.is_below({0, 1}));
		REQUIRE(!s2.is_below({1, 2}));
		REQUIRE(!s2.is_below({0, 0}));
		REQUIRE(!s2.is_below({5, -1}));
	}

	SECTION("isAbove") {
		SweepEvent s1b{.point = {1, 1}};
		SweepEvent s1{.point = {}, .otherEvent = &s1b, .left = true};
		SweepEvent s2b{.point = {0, 0}};
		SweepEvent s2{.point = {0, 1}, .otherEvent = &s2b};

		REQUIRE(!s1.is_above({0, 1}));
		REQUIRE(!s1.is_above({1, 2}));
		REQUIRE(s1.is_above({0, 0}));
		REQUIRE(s1.is_above({5, -1}));

		REQUIRE(s2.is_above({0, 1}));
		REQUIRE(s2.is_above({1, 2}));
		REQUIRE(s2.is_above({0, 0}));
		REQUIRE(s2.is_above({5, -1}));
	}

	SECTION("isVertical") {
		SweepEvent r1{.point = {0, 1}, .left = false};
		REQUIRE(SweepEvent{.point = {0, 0}, .otherEvent = &r1, .left = true}.is_vertical());

		SweepEvent r2{.point = {0.0001f, 1}, .left = false};
		REQUIRE(!SweepEvent{.point = {0, 0}, .otherEvent = &r2, .left = true}.is_vertical());
	}
}

TEST_CASE("analytical signed area", "martinez") {
	REQUIRE(signed_area({0, 0}, {0, 1}, {1, 1}) == -1);
	REQUIRE(signed_area({0, 1}, {0, 0}, {1, 0}) == 1);
	REQUIRE(signed_area({0, 0}, {1, 1}, {2, 2}) == 0);

	REQUIRE(signed_area({-1, 0}, {2, 3}, {0, 1}) == 0);
	REQUIRE(signed_area({2, 3}, {-1, 0}, {0, 1}) == 0);
}
