#ifndef _HYBRID_A_STAR_PTR_WRAPPER_H_
#define _HYBRID_A_STAR_PTR_WRAPPER_H_

#include <memory>

#define CLASS_SHARED(C) class C; using C##Ptr = std::shared_ptr<C>
#define CLASS_UNIQUE(C) class C; using C##Ptr = std::unique_ptr<C>

#endif