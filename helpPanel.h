#ifndef HELP_PANEL_H
#define HELP_PANEL_H

#include "chai3d.h"

using namespace chai3d;

class HelpPanel {
private:
    cCamera *camera;
    // panel that displays hotkeys
    cPanel *panel;
    // help panel header
    cLabel *header;
    // vector holding hotkey key labels
    std::vector<cLabel *> hotkeyKeys;
    // vector holding function key labels (must be separate for formatting)
    std::vector<cLabel *> hotkeyFunctions;
    bool showing; /* true if panel is displayed */

    void addHotkeyLabel(std::string keys, std::string function);
public:
    HelpPanel(cCamera *camera);
    void toggleDisplay();
    void resize(int width, int height);
};
 
#endif
