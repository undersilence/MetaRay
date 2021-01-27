#pragma once

#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>

// struct -> tuple
// copy from
// https://stackoverflow.com/questions/38561242/struct-to-from-stdtuple-conversion
// template <std::size_t N> struct to_tuple_t;

// template <> struct to_tuple_t<3> {
//   template <class S> auto operator()(S &&s) const {
//     auto [e0, e1, e2] = std::forward<S>(s);
//     return std::make_tuple(e0, e1, e2);
//   }
// };

// template <std::size_t N, class S> auto to_tuple(S &&s) { return
// to_tuple_t<N>{}(std::forward<S>(s)); }

// template <class S, std::size_t... Is, class Tup> S
// to_struct(std::index_sequence<Is...>, Tup &&tup) {
//   using std::get;
//   return {get<Is>(std::forward<Tup>(tup))...};
// }
// template <class S, class Tup> S to_struct(Tup &&tup) {
//   using T = std::remove_reference_t<Tup>;

//   return to_struct(std::make_index_sequence<std::tuple_size<T>{}>{},
//   std::forward<Tup>(tup));
// }
