//
// Created by imanol on 24/5/22.
//

#include "PickedColor.h"

PickedColor::PickedColor(std::string name, std::string file, unsigned int px, unsigned int py, unsigned int kernel,
                         unsigned int r, unsigned int g, unsigned int b) :
                         _colorName(name), _file(file), _kernel(kernel) {
    _pixel[0] = px;
    _pixel[1] = py;
    _colorValue[0] = r;
    _colorValue[1] = g;
    _colorValue[2] = b;

}
