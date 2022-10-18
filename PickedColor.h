//
// Created by imanol on 24/5/22.
//

#ifndef COLOR_RESTORATION_PICKEDCOLOR_H
#define COLOR_RESTORATION_PICKEDCOLOR_H


#include <string>

class PickedColor {
public:
    PickedColor(std::string name, std::string file, unsigned int px, unsigned int py, unsigned int kernel, unsigned int r, unsigned int g, unsigned int b);

    unsigned int red()      const { return _colorValue[0]; }
    unsigned int green()    const { return _colorValue[1]; }
    unsigned int blue()     const { return _colorValue[2]; }

    void setRed(unsigned int r)     { _colorValue[0] = r; }
    void setGreen(unsigned int g)   { _colorValue[1] = g; }
    void setBlue(unsigned int b)    { _colorValue[2] = b; }

private:
    std::string _colorName;
    std::string _file;
    unsigned int _pixel[2];
    unsigned int _kernel;
    unsigned int _colorValue[3];
};


#endif //COLOR_RESTORATION_PICKEDCOLOR_H
