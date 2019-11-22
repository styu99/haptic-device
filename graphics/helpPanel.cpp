#include "helpPanel.h"


// font for help screen
chai3d::cFontPtr helpFont = chai3d::NEW_CFONTCALIBRI32();


HelpPanel::HelpPanel(cCamera *cam){
    camera = cam;
    showing = false;

    // Initialize base panel
    cColorf panelColor = cColorf();
    panelColor.setBlueCadet();
    panel = new cPanel();
    panel->setColor(panelColor);
    panel->setSize(520, 500);
    camera->m_frontLayer->addChild(panel);
    panel->setShowPanel(false);

    // Initialize header
    cFontPtr headerFont = NEW_CFONTCALIBRI40();
    header = new cLabel(headerFont);
    header->m_fontColor.setBlack();
    header->setText("HOTKEYS AND INSTRUCTIONS");
    header->setShowPanel(false);

    // create hotkey labels
    addHotkeyLabel("f", "toggle fullscreen");
    addHotkeyLabel("q, ESC", "quit program");
    addHotkeyLabel("a", "anchor all atoms");
    addHotkeyLabel("u", "unanchor all atoms");
    addHotkeyLabel("ARROW KEYS", "rotate camera");
    addHotkeyLabel("[", "zoom in");
    addHotkeyLabel("]", "zoom out");
    addHotkeyLabel("r", "reset camera");
    addHotkeyLabel("s", "screenshot atoms");
    addHotkeyLabel("c", "save configuration to .con");
    addHotkeyLabel("SPACE", "freeze atoms");
    addHotkeyLabel("CTRL", "toggle help panel");

}

void HelpPanel::toggleDisplay(){
    showing = !showing;
    panel->setShowPanel(showing);
    if(showing){
        camera->m_frontLayer->addChild(header);
        for(int i = 0; i < hotkeyKeys.size(); i++){
          camera->m_frontLayer->addChild(hotkeyKeys[i]);
          camera->m_frontLayer->addChild(hotkeyFunctions[i]);
        }
    }else{
      camera->m_frontLayer->removeChild(header);
      for(int i = 0; i < hotkeyKeys.size(); i++){
        camera->m_frontLayer->removeChild(hotkeyKeys[i]);
        camera->m_frontLayer->removeChild(hotkeyFunctions[i]);
      }
    }
}

void HelpPanel::resize(int width, int height){
    panel->setLocalPos(width - 550, height - 530);
    header->setLocalPos(width - 490, height - 70);

    // rescale hotkey labels
    for (int i = 0; i < hotkeyKeys.size(); i++) {
      cLabel *tempKeyLabel = hotkeyKeys[i];
      cLabel *tempFuncLabel = hotkeyFunctions[i];
      tempKeyLabel->setLocalPos(width - 540, height - 130 - i * 25);
      tempFuncLabel->setLocalPos(width - 350, height - 130 - i * 25);
    }
}

//add help panel labels
void HelpPanel::addHotkeyLabel(std::string keys, std::string function){
  chai3d::cLabel *tempKeyLabel = new chai3d::cLabel(helpFont);
  chai3d::cLabel *tempFuncLabel = new chai3d::cLabel(helpFont);
  tempKeyLabel->m_fontColor.setBlack();
  tempFuncLabel->m_fontColor.setBlack();
  tempKeyLabel->setText(keys);
  tempFuncLabel->setText(function);
  tempKeyLabel->setShowPanel(false);
  tempFuncLabel->setShowPanel(false);
  hotkeyKeys.push_back(tempKeyLabel);
  hotkeyFunctions.push_back(tempFuncLabel);
}
