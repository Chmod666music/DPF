#include "DistrhoUI.hpp"

#include <cstring>

START_NAMESPACE_DISTRHO

class DrumCloudUI : public UI
{
public:
    DrumCloudUI()
        : UI(360, 90) // størrelse på vinduet
    {
    }

protected:
    void onDisplay() override
    {
        // Minimal UI: vi tegner ingenting.
        // (Du kan tilføje tekst/knap senere.)
    }

    bool onMouse(const MouseEvent& ev) override
    {
        // Venstre klik -> åbn host file dialog for state key "samplePath"
        if (ev.press && ev.button == 1)
        {
            requestStateFile("samplePath");
            return true;
        }
        return false;
    }

    void stateChanged(const char* key, const char* value) override
    {
        // Optional: her kunne vi vise stien i UI senere.
        (void)key;
        (void)value;
    }
};

UI* createUI()
{
    return new DrumCloudUI();
}

END_NAMESPACE_DISTRHO
