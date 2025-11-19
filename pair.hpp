#pragma once

namespace ZN {

template <typename K, typename V>
struct Pair {
	K first;
	V second;
};

template <typename K, typename V>
Pair(K, V) -> Pair<K, V>;

template <typename K, typename V> requires requires(K k, V v) { k == k; v == v; }
bool operator==(const Pair<K, V>& a, const Pair<K, V>& b) {
	return a.first == b.first && a.second == b.second;
}

template <typename K, typename V> requires requires(K k, V v) { k != k; v != v; }
bool operator!=(const Pair<K, V>& a, const Pair<K, V>& b) {
	return a.first == b.first && a.second == b.second;
}

}

using ZN::Pair;

