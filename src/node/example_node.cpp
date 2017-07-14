/*
 * test.cpp
 *
 *  Created on: 20.06.2017
 *      Author: burrimi
 */

#include <iostream>
#include <typeinfo>
using namespace std;

class Base {
 public:
  typedef std::shared_ptr<Base> Ptr;
  virtual void func();
};

class Derived : public Base {
 public:
  virtual void func() {
    return;
  }
};

class AnotherDerived : public Base {
 public:
  virtual void func() {
    return;
  }
  virtual void func(int i) {
    std::cout << i << std::endl;
  }
};

int main() {
  Base* dp = new Derived;
  Base* adp = new AnotherDerived;
  cout << typeid(*dp).name() << endl;
  cout << typeid(*adp).name() << endl;
  static_cast<AnotherDerived*>(adp)->func(3);

  Base::Ptr test = make_shared<Derived>();

  return 0;
}
