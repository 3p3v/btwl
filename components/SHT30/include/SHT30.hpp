#pragma once

#include "SHT30Readings.hpp"
#include "SHT30Parser.hpp"

/* User API */
class SHT30 : public SHT30Cmds, public SHT30Parser {
public:
    SHT30();
    virtual ~SHT30();
};