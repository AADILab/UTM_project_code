// Copyright 2016 Carrie Rebhuhn
#ifndef STL_EASYSTL_H_
#define STL_EASYSTL_H_
#include <algorithm>
#include <list>
#include <vector>

namespace easystl {

enum {ELEMENT_NOT_FOUND=200};

template <class ptr>
void clear(std::vector<ptr*> &v) {
  while (v.size()) {
    delete v.back();
    v.pop_back();
  }
}

template <class ptr>
void clear(std::list<ptr*> &v) {
  while (v.size()) {
    delete v.back();
    v.pop_back();
  }
}

//! Remove-erase-if idiom
template <class Container, class UnaryPredicate>
void remove_erase_if(Container* stl, UnaryPredicate pred) {
  stl->erase(std::remove_if(stl->begin(), stl->end(), pred), stl->end());
}

template <class Container, class T>
void remove_element(Container* stl, T el) {
  if (stl->empty()) throw ELEMENT_NOT_FOUND;
  auto it = std::find(stl->begin(), stl->end(), el);
  if (it != stl->end())
    stl->erase(it);
  else throw ELEMENT_NOT_FOUND;
}
}  // namespace easystl
#endif  // STL_EASYSTL_H_
