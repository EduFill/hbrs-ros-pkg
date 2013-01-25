#ifndef COLOR_H
#define COLOR_H

#include <pcl16/point_types.h>
#include <std_msgs/ColorRGBA.h>

namespace hbrs
{

namespace visualization
{

struct Color
{

  uint8_t r;
  uint8_t g;
  uint8_t b;

  Color(float nr, float ng, float nb) : r(nr * 255), g(ng * 255), b(nb * 255) { }

  Color(uint8_t nr, uint8_t ng, uint8_t nb) : r(nr), g(ng), b(nb) { }

  enum Name
  {
    SALMON,
    TEAL,
    DEEP_PINK,
    SANGRIA,
    SEA_BLUE,
    SEA_GREEN,
    SCARLET,
    PUMPKIN,
    JASMINE,
    IVORY,
    GAINSBORO,
  };

  Color(Name name)
  {
    switch (name)
    {
      case SALMON:        r = 0xFA, g = 0x80, b = 0x72; break;
      case TEAL:          r = 0x00, g = 0x80, b = 0x80; break;
      case DEEP_PINK:     r = 0xFF, g = 0x14, b = 0x93; break;
      case SANGRIA:       r = 0x92, g = 0x00, b = 0x0A; break;
      case SEA_BLUE:      r = 0x00, g = 0x69, b = 0x94; break;
      case SEA_GREEN:     r = 0x2E, g = 0x8B, b = 0x57; break;
      case SCARLET:       r = 0xFF, g = 0x24, b = 0x00; break;
      case PUMPKIN:       r = 0xFF, g = 0x75, b = 0x18; break;
      case JASMINE:       r = 0xF8, g = 0xDE, b = 0x7E; break;
      case IVORY:         r = 0xFF, g = 0xFF, b = 0xF0; break;
      case GAINSBORO:     r = 0xDC, g = 0xDC, b = 0xDC; break;
      default:            r = 0xFF, g = 0xFF, b = 0xFF; break;
    }
  }

  operator float() const
  {
    pcl16::PointXYZRGB point(r, g, b);
    return point.rgb;
  }

  operator std_msgs::ColorRGBA() const
  {
    std_msgs::ColorRGBA color;
    color.r = r / 255.0;
    color.g = g / 255.0;
    color.b = b / 255.0;
    color.a = 1.0;
    return color;
  }

};

}

}

#endif /* COLOR_H */

