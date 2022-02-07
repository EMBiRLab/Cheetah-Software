#ifndef COLOR_H
#define COLOR_H

#pragma once

#include <ostream>

namespace Color {
  enum Code : int {
    FG_BLACK    = 30,
    FG_RED      = 31,
    FG_GREEN    = 32,
    FG_YELLOW   = 33,
    FG_BLUE     = 34,
    FG_DEFAULT  = 39,
    BG_RED      = 41,
    BG_GREEN    = 42,
    BG_YELLOW   = 43,
    BG_BLUE     = 44,
    BG_WHITE    = 47,
    BG_DEFAULT  = 49
  };
  class Modifier {
    Code code;
  public:
    Modifier(Code pCode) : code(pCode) {}
    friend std::ostream&
    operator<<(std::ostream& os, const Modifier& mod) {
      return os << "\033[" << int(mod.code) << "m";
    }
    Modifier& operator=(const Modifier& mod) {
      this->code = mod.code;
      return *this;
    }
    // kinda goofy design here but it works :shrug:
    Color::Modifier fg_blk() {return Color::Modifier(Color::FG_BLACK);}
    Color::Modifier fg_red() {return Color::Modifier(Color::FG_RED);}
    Color::Modifier fg_grn() {return Color::Modifier(Color::FG_GREEN);}
    Color::Modifier fg_blu() {return Color::Modifier(Color::FG_BLUE);}
    Color::Modifier fg_yel() {return Color::Modifier(Color::FG_YELLOW);}
    Color::Modifier fg_def() {return Color::Modifier(Color::FG_DEFAULT);}

    Color::Modifier bg_red() {return Color::Modifier(Color::BG_RED);}
    Color::Modifier bg_grn() {return Color::Modifier(Color::BG_GREEN);}
    Color::Modifier bg_blu() {return Color::Modifier(Color::BG_BLUE);}
    Color::Modifier bg_yel() {return Color::Modifier(Color::BG_YELLOW);}
    Color::Modifier bg_wht() {return Color::Modifier(Color::BG_WHITE);}
    Color::Modifier bg_def() {return Color::Modifier(Color::BG_DEFAULT);}
  };
}

// namespace CMod {
//   Color::Modifier fg_blk() {return Color::Modifier(Color::FG_BLACK);}
//   Color::Modifier fg_red() {return Color::Modifier(Color::FG_RED);}
//   Color::Modifier fg_grn() {return Color::Modifier(Color::FG_GREEN);}
//   Color::Modifier fg_blu() {return Color::Modifier(Color::FG_BLUE);}
//   Color::Modifier fg_yel() {return Color::Modifier(Color::FG_YELLOW);}
//   Color::Modifier fg_def() {return Color::Modifier(Color::FG_DEFAULT);}

//   Color::Modifier bg_red() {return Color::Modifier(Color::BG_RED);}
//   Color::Modifier bg_grn() {return Color::Modifier(Color::BG_GREEN);}
//   Color::Modifier bg_blu() {return Color::Modifier(Color::BG_BLUE);}
//   Color::Modifier bg_yel() {return Color::Modifier(Color::BG_YELLOW);}
//   Color::Modifier bg_wht() {return Color::Modifier(Color::BG_WHITE);}
//   Color::Modifier bg_def() {return Color::Modifier(Color::BG_DEFAULT);}
// }

#endif