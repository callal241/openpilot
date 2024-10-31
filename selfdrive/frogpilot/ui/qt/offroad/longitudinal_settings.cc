#include "selfdrive/frogpilot/ui/qt/offroad/longitudinal_settings.h"

FrogPilotLongitudinalPanel::FrogPilotLongitudinalPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  const std::vector<std::tuple<QString, QString, QString, QString>> longitudinalToggles {
    {"ConditionalExperimental", tr("Conditional Experimental Mode"), tr("Automatically switch to 'Experimental Mode' when specific conditions are met."), "../frogpilot/assets/toggle_icons/icon_conditional.png"},
    {"CESpeed", tr("Below"), tr("Triggers 'Experimental Mode' when driving below the set speed without a lead vehicle."), ""},
    {"CECurves", tr("Curve Detected Ahead"), tr("Triggers 'Experimental Mode' when a curve is detected in the road ahead."), ""},
    {"CELead", tr("Lead Detected Ahead"), tr("Triggers 'Experimental Mode' when a slower or stopped vehicle is detected ahead."), ""},
    {"CENavigation", tr("Navigation Data"), tr("Triggers 'Experimental Mode' based on navigation data, such as upcoming intersections or turns."), ""},
    {"CEModelStopTime", tr("openpilot Wants to Stop In"), tr("Triggers 'Experimental Mode' when openpilot wants to stop such as for a stop sign or red light."), ""},
    {"CESignalSpeed", tr("Turn Signal Below"), tr("Triggers 'Experimental Mode' when using turn signals below the set speed."), ""},
    {"HideCEMStatusBar", tr("Hide the Status Bar"), tr("Hides status bar for 'Conditional Experimental Mode'."), ""},

    {"CurveSpeedControl", tr("Curve Speed Control"), tr("Automatically slow down for curves detected ahead or through the downloaded maps."), "../frogpilot/assets/toggle_icons/icon_speed_map.png"},
    {"CurveDetectionMethod", tr("Curve Detection Method"), tr("Uses data from either the downloaded maps or the model to determine where curves are."), ""},
    {"MTSCCurvatureCheck", tr("Curve Detection Failsafe"), tr("Triggers 'Curve Speed Control' only when a curve is detected with the model as well when using the 'Map Based' method."), ""},
    {"CurveSensitivity", tr("Curve Detection Sensitivity"), tr("Controls how sensitive openpilot is to detecting curves. Higher values trigger earlier responses at the risk of triggering too often, while lower values increase confidence at the risk of triggering too infrequently."), ""},
    {"TurnAggressiveness", tr("Speed Aggressiveness"), tr("Controls how aggressive openpilot takes turns. Higher values result in faster turns, while lower values result in slower turns."), ""},
    {"DisableCurveSpeedSmoothing", tr("Disable Speed Value Smoothing In the UI"), tr("Speed value smoothing is disabled in the UI to instead display the exact speed requested by the curve control."), ""},

    {"CustomPersonalities", tr("Customize Driving Personalities"), tr("Customize the personality profiles to suit your driving style."), "../frogpilot/assets/toggle_icons/icon_personality.png"},
    {"TrafficPersonalityProfile", tr("Traffic Personality"), tr("Customizes the 'Traffic' personality profile, tailored for navigating through traffic."), "../frogpilot/assets/stock_theme/distance_icons/traffic.png"},
    {"TrafficFollow", tr("Following Distance"), tr("Controls the minimum following distance in 'Traffic' mode. openpilot will automatically dynamically between this value and the 'Aggressive' profile distance based on your current speed."), ""},
    {"TrafficJerkAcceleration", tr("Acceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in acceleration in 'Traffic' mode. Higher values result in smoother, more gradual acceleration, while lower values allow for quicker, more responsive changes that may feel abrupt."), ""},
    {"TrafficJerkDeceleration", tr("Deceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in deceleration in 'Traffic' mode. Higher values result in smoother, more gradual deceleration, while lower values allow for quicker, more responsive changes that may feel abrupt."), ""},
    {"TrafficJerkDanger", tr("Safety Distance Sensitivity"), tr("Adjusts how cautious openpilot is around other vehicles or obstacles in 'Traffic' mode. Higher values increase following distances and prioritize safety, leading to more cautious driving, while lower values allow for closer following but may reduce reaction time."), ""},
    {"TrafficJerkSpeed", tr("Speed Increase Responsiveness"), tr("Controls how quickly openpilot increases speed in 'Traffic' mode. Higher values ensure smoother, more gradual speed changes when accelerating, while lower values allow for quicker, more responsive changes that may feel abrupt."), ""},
    {"TrafficJerkSpeedDecrease", tr("Speed Decrease Responsiveness"), tr("Sets how quickly openpilot adjusts to decreasing speeds in 'Traffic' mode. Higher values ensure smoother, more gradual speed changes when slowing down, while lower values allow for quicker, more responsive changes that may feel abrupt."), ""},
    {"ResetTrafficPersonality", tr("Reset Settings"), tr("Restores the 'Traffic Mode' settings to their default values."), ""},

    {"AggressivePersonalityProfile", tr("Aggressive Personality"), tr("Customize the 'Aggressive' personality profile, designed for a more assertive driving style."), "../frogpilot/assets/stock_theme/distance_icons/aggressive.png"},
    {"AggressiveFollow", tr("Following Distance"), tr("Sets the following distance for 'Aggressive' mode. This determines roughly how many seconds you'll follow behind the car ahead.\n\nDefault: 1.25 seconds."), ""},
    {"AggressiveJerkAcceleration", tr("Acceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in acceleration in 'Aggressive' mode. Higher values result in smoother, more gradual acceleration, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 0.5."), ""},
    {"AggressiveJerkDeceleration", tr("Deceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in deceleration in 'Aggressive' mode. Higher values result in smoother, more gradual deceleration, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 0.5."), ""},
    {"AggressiveJerkDanger", tr("Safety Distance Sensitivity"), tr("Adjusts how cautious openpilot is around other vehicles or obstacles in 'Aggressive' mode. Higher values increase following distances and prioritize safety, leading to more cautious driving, while lower values allow for closer following but may reduce reaction time.\n\nDefault: 1.0."), ""},
    {"AggressiveJerkSpeed", tr("Speed Increase Responsiveness"), tr("Controls how quickly openpilot increases speed in 'Aggressive' mode. Higher values ensure smoother, more gradual speed changes when accelerating, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 0.5."), ""},
    {"AggressiveJerkSpeedDecrease", tr("Speed Decrease Responsiveness"), tr("Sets how quickly openpilot adjusts to decreasing speeds in 'Aggressive' mode. Higher values ensure smoother, more gradual speed changes when slowing down, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 0.5."), ""},
    {"ResetAggressivePersonality", tr("Reset Settings"), tr("Restores the 'Aggressive' settings to their default values."), ""},

    {"StandardPersonalityProfile", tr("Standard Personality"), tr("Customize the 'Standard' personality profile, optimized for balanced driving."), "../frogpilot/assets/stock_theme/distance_icons/standard.png"},
    {"StandardFollow", tr("Following Distance"), tr("Set the following distance for 'Standard' mode. This determines roughly how many seconds you'll follow behind the car ahead.\n\nDefault: 1.45 seconds."), ""},
    {"StandardJerkAcceleration", tr("Acceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in acceleration in 'Standard' mode. Higher values result in smoother, more gradual acceleration, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"StandardJerkDeceleration", tr("Deceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in deceleration in 'Standard' mode. Higher values result in smoother braking, while lower values allow for quicker, more immediate braking that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"StandardJerkDanger", tr("Safety Distance Sensitivity"), tr("Adjusts how cautious openpilot is around other vehicles or obstacles in 'Standard' mode. Higher values increase following distances and prioritize safety, leading to more cautious driving, while lower values allow for closer following but may reduce reaction time.\n\nDefault: 1.0."), ""},
    {"StandardJerkSpeed", tr("Speed Increase Responsiveness"), tr("Controls how quickly openpilot increases speed in 'Standard' mode. Higher values ensure smoother, more gradual speed changes when accelerating, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"StandardJerkSpeedDecrease", tr("Speed Decrease Responsiveness"), tr("Sets how quickly openpilot adjusts to decreasing speeds in 'Standard' mode. Higher values ensure smoother, more gradual speed changes when slowing down, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"ResetStandardPersonality", tr("Reset Settings"), tr("Restores the 'Standard' settings to their default values."), ""},

    {"RelaxedPersonalityProfile", tr("Relaxed Personality"), tr("Customize the 'Relaxed' personality profile, ideal for a more laid-back driving style."), "../frogpilot/assets/stock_theme/distance_icons/relaxed.png"},
    {"RelaxedFollow", tr("Following Distance"), tr("Set the following distance for 'Relaxed' mode. This determines roughly how many seconds you'll follow behind the car ahead.\n\nDefault: 1.75 seconds."), ""},
    {"RelaxedJerkAcceleration", tr("Acceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in acceleration in 'Relaxed' mode. Higher values result in smoother, more gradual acceleration, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"RelaxedJerkDeceleration", tr("Deceleration Sensitivity"), tr("Controls how sensitive openpilot is to changes in deceleration in 'Relaxed' mode. Higher values result in smoother braking, while lower values allow for quicker, more immediate braking that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"RelaxedJerkDanger", tr("Safety Distance Sensitivity"), tr("Adjusts how cautious openpilot is around other vehicles or obstacles in 'Relaxed' mode. Higher values increase following distances and prioritize safety, leading to more cautious driving, while lower values allow for closer following but may reduce reaction time.\n\nDefault: 1.0."), ""},
    {"RelaxedJerkSpeed", tr("Speed Increase Responsiveness"), tr("Controls how quickly openpilot increases speed in 'Relaxed' mode. Higher values ensure smoother, more gradual speed changes when accelerating, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"RelaxedJerkSpeedDecrease", tr("Speed Decrease Responsiveness"), tr("Sets how quickly openpilot adjusts to decreasing speeds in 'Relaxed' mode. Higher values ensure smoother, more gradual speed changes when slowing down, while lower values allow for quicker, more responsive changes that may feel abrupt.\n\nDefault: 1.0."), ""},
    {"ResetRelaxedPersonality", tr("Reset Settings"), tr("Restores the 'Relaxed' settings to their default values."), ""},

    {"ExperimentalModeActivation", tr("Experimental Mode Activation"), tr("Toggle 'Experimental Mode' on/off using either the steering wheel buttons or screen.\n\nThis overrides 'Conditional Experimental Mode'."), "../assets/img_experimental_white.svg"},
    {"ExperimentalModeViaLKAS", tr("Click the LKAS Button"), tr("Toggles 'Experimental Mode' by pressing the 'LKAS' button on the steering wheel."), ""},
    {"ExperimentalModeViaTap", tr("Double-Tap the Screen"), tr("Toggles 'Experimental Mode' by double-tapping the onroad UI within a 0.5 second period."), ""},
    {"ExperimentalModeViaDistance", tr("Long Press the Distance Button"), tr("Toggles 'Experimental Mode' by holding down the 'distance' button on the steering wheel for 0.5 seconds."), ""},

    {"LongitudinalTune", tr("Longitudinal Tuning"), tr("Settings that control how openpilot manages speed and acceleration."), "../frogpilot/assets/toggle_icons/icon_longitudinal_tune.png"},
    {"AccelerationProfile", tr("Acceleration Profile"), tr("Enables either a sporty or eco-friendly acceleration rate. 'Sport+' aims to make openpilot accelerate as fast as possible."), ""},
    {"DecelerationProfile", tr("Deceleration Profile"), tr("Enables either a sporty or eco-friendly deceleration rate."), ""},
    {"HumanAcceleration", tr("Human-Like Acceleration"), tr("Uses the lead's acceleration rate when at a takeoff and ramps off the acceleration rate when approaching the maximum set speed for a more 'human-like' driving experience."), ""},
    {"HumanFollowing", tr("Human-Like Following Distance"), tr("Dynamically adjusts the following distance when approaching slower or stopped vehicles for a more 'human-like' driving experience."), ""},
    {"IncreasedStoppedDistance", tr("Increase Stopped Distance"), tr("Increases the distance to stop behind vehicles."), ""},
    {"LeadDetectionThreshold", tr("Lead Detection Confidence"), tr("Controls how sensitive openpilot is to detecting vehicles ahead. A lower value can help detect vehicles sooner and from farther away, but increases the chance openpilot mistakes other objects for vehicles."), ""},
    {"MaxDesiredAcceleration", tr("Maximum Acceleration Rate"), tr("Sets a cap on how fast openpilot can accelerate."), ""},

    {"QOLLongitudinal", tr("Quality of Life Improvements"), tr("Miscellaneous longitudinal focused features to improve your overall openpilot experience."), "../frogpilot/assets/toggle_icons/quality_of_life.png"},
    {"CustomCruise", tr("Cruise Increase Interval"), tr("Controls the interval used when increasing the cruise control speed."), ""},
    {"CustomCruiseLong", tr("Custom Cruise Interval (Long Press)"), tr("Controls the interval used when increasing the cruise control speed while holding down the button for 0.5+ seconds."), ""},
    {"ForceStandstill", tr("Force Keep openpilot in the Standstill State"), tr("Keeps openpilot in the 'standstill' state until the gas pedal or 'resume' button is pressed."), ""},
    {"ForceStops", tr("Force Stop for 'Detected' Stop Lights/Signs"), tr("Forces a stop whenever openpilot 'detects' a potential red light/stop sign to prevent it from running the red light/stop sign."), ""},
    {"SetSpeedOffset", tr("Set Speed Offset"), tr("Controls how much higher or lower the set speed should be compared to your current set speed. For example, if you prefer to drive 5 mph above the speed limit, this setting will automatically add that difference when you adjust your set speed."), ""},
    {"MapGears", tr("Map Accel/Decel to Gears"), tr("Maps the acceleration and deceleration profiles to your car's 'Eco' or 'Sport' gear modes."), ""},
    {"ReverseCruise", tr("Reverse Cruise Increase"), tr("Reverses the long press cruise increase feature to increase the max speed by 5 mph instead of 1 on short presses."), ""},

    {"SpeedLimitController", tr("Speed Limit Controller"), tr("Automatically adjust your max speed to match the speed limit using downloaded 'Open Street Maps' data, 'Navigate on openpilot', or your car's dashboard (Toyota/Lexus/HKG only)."), "../assets/offroad/icon_speed_limit.png"},
    {"SLCConfirmation", tr("Confirm New Speed Limits"), tr("Enables manual confirmations before using a new speed limit."), ""},
    {"SLCFallback", tr("Fallback Method"), tr("Controls what happens when no speed limit data is available."), ""},
    {"SLCOverride", tr("Override Method"), tr("Controls how the current speed limit is overriden.\n\n"), ""},
    {"SLCQOL", tr("Quality of Life Improvements"), tr("Miscellaneous 'Speed Limit Controller' focused features to improve your overall openpilot experience."), ""},
    {"ForceMPHDashboard", tr("Force MPH Readings from Dashboard"), tr("Forces speed limit readings from the dashboard to MPH if it normally displays them in KPH."), ""},
    {"SLCLookaheadHigher", tr("Prepare for Higher Speed Limits"), tr("Sets a lookahead value to prepare for upcoming higher speed limits when using downloaded map data."), ""},
    {"SLCLookaheadLower", tr("Prepare for Lower Speed Limits"), tr("Sets a lookahead value to prepare for upcoming lower speed limits when using downloaded map data."), ""},
    {"SetSpeedLimit", tr("Set Speed to Current Limit"), tr("Sets your max speed to match the current speed limit when enabling openpilot."), ""},
    {"SLCPriority", tr("Speed Limit Source Priority"), tr("Sets the order of priority for speed limit data sources."), ""},
    {"SLCOffsets", tr("Speed Limit Offsets"), tr("Set speed limit offsets to drive over the posted speed limit."), ""},
    {"Offset1", tr("Speed Limit Offset (0-34 mph)"), tr("Sets the speed limit offset for speeds between 0 and 34 mph."), ""},
    {"Offset2", tr("Speed Limit Offset (35-54 mph)"), tr("Sets the speed limit offset for speeds between 35 and 54 mph."), ""},
    {"Offset3", tr("Speed Limit Offset (55-64 mph)"), tr("Sets the speed limit offset for speeds between 55 and 64 mph."), ""},
    {"Offset4", tr("Speed Limit Offset (65-99 mph)"), tr("Sets the speed limit offset for speeds between 65 and 99 mph."), ""},
  };

  for (const auto &[param, title, desc, icon] : longitudinalToggles) {
    AbstractControl *longitudinalToggle;

    if (param == "CustomPersonalities") {
      FrogPilotParamManageControl *customPersonalitiesToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(customPersonalitiesToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        showToggles(customDrivingPersonalityKeys);
      });
      longitudinalToggle = customPersonalitiesToggle;
    } else if (param == "ResetTrafficPersonality" || param == "ResetAggressivePersonality" || param == "ResetStandardPersonality" || param == "ResetRelaxedPersonality") {
      FrogPilotButtonsControl *profileBtn = new FrogPilotButtonsControl(title, desc, {tr("Reset")});
      longitudinalToggle = profileBtn;
    } else if (param == "TrafficPersonalityProfile") {
      FrogPilotParamManageControl *trafficPersonalityToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(trafficPersonalityToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        customPersonalityOpen = true;
        openSubParentToggle();
        showToggles(trafficPersonalityKeys);
      });
      longitudinalToggle = trafficPersonalityToggle;
    } else if (param == "AggressivePersonalityProfile") {
      FrogPilotParamManageControl *aggressivePersonalityToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(aggressivePersonalityToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        customPersonalityOpen = true;
        openSubParentToggle();
        showToggles(aggressivePersonalityKeys);
      });
      longitudinalToggle = aggressivePersonalityToggle;
    } else if (param == "StandardPersonalityProfile") {
      FrogPilotParamManageControl *standardPersonalityToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(standardPersonalityToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        customPersonalityOpen = true;
        openSubParentToggle();
        showToggles(standardPersonalityKeys);
      });
      longitudinalToggle = standardPersonalityToggle;
    } else if (param == "RelaxedPersonalityProfile") {
      FrogPilotParamManageControl *relaxedPersonalityToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(relaxedPersonalityToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        customPersonalityOpen = true;
        openSubParentToggle();
        showToggles(relaxedPersonalityKeys);
      });
      longitudinalToggle = relaxedPersonalityToggle;
    } else if (trafficPersonalityKeys.find(param) != trafficPersonalityKeys.end() ||
               aggressivePersonalityKeys.find(param) != aggressivePersonalityKeys.end() ||
               standardPersonalityKeys.find(param) != standardPersonalityKeys.end() ||
               relaxedPersonalityKeys.find(param) != relaxedPersonalityKeys.end()) {
      if (param == "TrafficFollow" || param == "AggressiveFollow" || param == "StandardFollow" || param == "RelaxedFollow") {
        if (param == "TrafficFollow") {
          longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.5, 5, tr(" seconds"), std::map<int, QString>(), 0.01);
        } else {
          longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 5, tr(" seconds"), std::map<int, QString>(), 0.01);
        }
      } else {
        longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 500, "%");
      }

    } else if (param == "ConditionalExperimental") {
      FrogPilotParamManageControl *conditionalExperimentalToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(conditionalExperimentalToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        showToggles(conditionalExperimentalKeys);
      });
      longitudinalToggle = conditionalExperimentalToggle;
    } else if (param == "CESpeed") {
      FrogPilotParamValueControl *CESpeed = new FrogPilotParamValueControl(param, title, desc, icon, 0, 99, tr("mph"), std::map<int, QString>(), 1.0, true);
      FrogPilotParamValueControl *CESpeedLead = new FrogPilotParamValueControl("CESpeedLead", tr(" With Lead"), tr("Switches to 'Experimental Mode' when driving below the set speed with a lead vehicle."), icon, 0, 99, tr("mph"), std::map<int, QString>(), 1.0, true);
      FrogPilotDualParamControl *conditionalSpeeds = new FrogPilotDualParamControl(CESpeed, CESpeedLead);
      longitudinalToggle = reinterpret_cast<AbstractControl*>(conditionalSpeeds);
    } else if (param == "CECurves") {
      std::vector<QString> curveToggles{"CECurvesLead"};
      std::vector<QString> curveToggleNames{tr("With Lead")};
      longitudinalToggle = new FrogPilotButtonToggleControl(param, title, desc, curveToggles, curveToggleNames);
    } else if (param == "CELead") {
      std::vector<QString> leadToggles{"CESlowerLead", "CEStoppedLead"};
      std::vector<QString> leadToggleNames{tr("Slower Lead"), tr("Stopped Lead")};
      longitudinalToggle = new FrogPilotButtonToggleControl(param, title, desc, leadToggles, leadToggleNames);
    } else if (param == "CENavigation") {
      std::vector<QString> navigationToggles{"CENavigationIntersections", "CENavigationTurns", "CENavigationLead"};
      std::vector<QString> navigationToggleNames{tr("Intersections"), tr("Turns"), tr("With Lead")};
      longitudinalToggle = new FrogPilotButtonToggleControl(param, title, desc, navigationToggles, navigationToggleNames);
    } else if (param == "CEModelStopTime") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 10, tr(" seconds"), {{0, "Off"}});
    } else if (param == "CESignalSpeed") {
      std::vector<QString> ceSignalToggles{"CESignalLaneDetection"};
      std::vector<QString> ceSignalToggleNames{"Only For Detected Lanes"};
      longitudinalToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 0, 99, tr("mph"), std::map<int, QString>(), 1.0, ceSignalToggles, ceSignalToggleNames, true);

    } else if (param == "CurveSpeedControl") {
      FrogPilotParamManageControl *curveControlToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(curveControlToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        curveDetectionBtn->setEnabledButtons(0, QDir("/data/media/0/osm/offline").exists());
        curveDetectionBtn->setCheckedButton(0, params.getBool("MTSCEnabled"));
        curveDetectionBtn->setCheckedButton(1, params.getBool("VisionTurnControl"));

        std::set<QString> modifiedCurveSpeedKeys = curveSpeedKeys;

        if (!params.getBool("MTSCEnabled")) {
          modifiedCurveSpeedKeys.erase("MTSCCurvatureCheck");
        }

        if (customizationLevel != 2) {
          modifiedCurveSpeedKeys.erase("CurveSensitivity");
          modifiedCurveSpeedKeys.erase("MTSCCurvatureCheck");
          modifiedCurveSpeedKeys.erase("TurnAggressiveness");
        }

        showToggles(modifiedCurveSpeedKeys);
      });
      longitudinalToggle = curveControlToggle;
    } else if (param == "CurveDetectionMethod") {
      curveDetectionBtn = new FrogPilotButtonsControl(title, desc, {tr("Map Based"), tr("Vision")}, true, false);
      QObject::connect(curveDetectionBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        bool mtscEnabled = params.getBool("MTSCEnabled");
        bool vtscEnabled = params.getBool("VisionTurnControl");

        if (id == 0) {
          if (mtscEnabled && !vtscEnabled) {
            curveDetectionBtn->setCheckedButton(0, true);
            return;
          }

          params.putBool("MTSCEnabled", !mtscEnabled);
          curveDetectionBtn->setCheckedButton(0, !mtscEnabled);

          std::set<QString> modifiedCurveSpeedKeys = curveSpeedKeys;

          if (mtscEnabled) {
            modifiedCurveSpeedKeys.erase("MTSCCurvatureCheck");
          }

          showToggles(modifiedCurveSpeedKeys);
        } else if (id == 1) {
          if (vtscEnabled && !mtscEnabled) {
            curveDetectionBtn->setCheckedButton(1, true);
            return;
          }

          params.putBool("VisionTurnControl", !vtscEnabled);
          curveDetectionBtn->setCheckedButton(1, !vtscEnabled);
        }
      });
      longitudinalToggle = curveDetectionBtn;
    } else if (param == "CurveSensitivity" || param == "TurnAggressiveness") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 200, "%");

    } else if (param == "ExperimentalModeActivation") {
      FrogPilotParamManageControl *experimentalModeActivationToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(experimentalModeActivationToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        std::set<QString> modifiedExperimentalModeActivationKeys = experimentalModeActivationKeys;

        if (isSubaru || (params.getBool("AlwaysOnLateral") && params.getBool("AlwaysOnLateralLKAS"))) {
          modifiedExperimentalModeActivationKeys.erase("ExperimentalModeViaLKAS");
        }

        showToggles(modifiedExperimentalModeActivationKeys);
      });
      longitudinalToggle = experimentalModeActivationToggle;

    } else if (param == "LongitudinalTune") {
      FrogPilotParamManageControl *longitudinalTuneToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(longitudinalTuneToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        std::set<QString> modifiedLongitudinalTuneKeys = longitudinalTuneKeys;

        if (customizationLevel == 0) {
          modifiedLongitudinalTuneKeys.erase("HumanAcceleration");
          modifiedLongitudinalTuneKeys.erase("HumanFollowing");
          modifiedLongitudinalTuneKeys.erase("IncreasedStoppedDistance");
          modifiedLongitudinalTuneKeys.erase("LeadDetectionThreshold");
          modifiedLongitudinalTuneKeys.erase("MaxDesiredAcceleration");
        } else if (customizationLevel == 1) {
          modifiedLongitudinalTuneKeys.erase("LeadDetectionThreshold");
          modifiedLongitudinalTuneKeys.erase("MaxDesiredAcceleration");
        }

        showToggles(modifiedLongitudinalTuneKeys);
      });
      longitudinalToggle = longitudinalTuneToggle;
    } else if (param == "AccelerationProfile") {
      std::vector<QString> accelerationProfiles{tr("Standard"), tr("Eco"), tr("Sport"), tr("Sport+")};
      ButtonParamControl *accelerationProfileToggle = new ButtonParamControl(param, title, desc, icon, accelerationProfiles);
      longitudinalToggle = accelerationProfileToggle;
    } else if (param == "DecelerationProfile") {
      std::vector<QString> decelerationProfiles{tr("Standard"), tr("Eco"), tr("Sport")};
      ButtonParamControl *decelerationProfileToggle = new ButtonParamControl(param, title, desc, icon, decelerationProfiles);
      longitudinalToggle = decelerationProfileToggle;
    } else if (param == "IncreasedStoppedDistance") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 10, tr(" feet"));
    } else if (param == "LeadDetectionThreshold") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 99, "%");
    } else if (param == "MaxDesiredAcceleration") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.1, 4.0, "m/s", std::map<int, QString>(), 0.1);

    } else if (param == "QOLLongitudinal") {
      FrogPilotParamManageControl *qolLongitudinalToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(qolLongitudinalToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        std::set<QString> modifiedQolKeys = qolKeys;

        if (!hasPCMCruise) {
          modifiedQolKeys.erase("ReverseCruise");
        } else {
          modifiedQolKeys.erase("CustomCruise");
          modifiedQolKeys.erase("CustomCruiseLong");
          modifiedQolKeys.erase("SetSpeedOffset");
        }

        if (!(isGM || isHKGCanFd || isToyota)) {
          modifiedQolKeys.erase("MapGears");
        }

        if (customizationLevel != 2) {
          modifiedQolKeys.erase("ForceStandstill");
          modifiedQolKeys.erase("ForceStops");
          modifiedQolKeys.erase("SetSpeedOffset");
        }

        showToggles(modifiedQolKeys);
      });
      longitudinalToggle = qolLongitudinalToggle;
    } else if (param == "CustomCruise") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 99, tr("mph"));
    } else if (param == "CustomCruiseLong") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 99, tr("mph"));
    } else if (param == "MapGears") {
      std::vector<QString> mapGearsToggles{"MapAcceleration", "MapDeceleration"};
      std::vector<QString> mapGearsToggleNames{tr("Acceleration"), tr("Deceleration")};
      longitudinalToggle = new FrogPilotButtonToggleControl(param, title, desc, mapGearsToggles, mapGearsToggleNames);
    } else if (param == "SetSpeedOffset") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 99, tr("mph"));

    } else if (param == "SpeedLimitController") {
      FrogPilotParamManageControl *speedLimitControllerToggle = new FrogPilotParamManageControl(param, title, desc, icon);
      QObject::connect(speedLimitControllerToggle, &FrogPilotParamManageControl::manageButtonClicked, [this]() {
        bool slcLower = params.getBool("SLCConfirmationLower");
        bool slcHigher = params.getBool("SLCConfirmationHigher");

        slcConfirmationBtn->setCheckedButton(0, slcLower);
        slcConfirmationBtn->setCheckedButton(1, slcHigher);
        slcConfirmationBtn->setCheckedButton(2, !(slcLower || slcHigher));

        std::set<QString> modifiedSpeedLimitControllerKeys = speedLimitControllerKeys;

        if (customizationLevel != 2) {
          modifiedSpeedLimitControllerKeys.erase("SLCFallback");
          modifiedSpeedLimitControllerKeys.erase("SLCOverride");
          modifiedSpeedLimitControllerKeys.erase("SLCPriority");
          modifiedSpeedLimitControllerKeys.erase("SLCQOL");
        }

        showToggles(modifiedSpeedLimitControllerKeys);

        slcOpen = true;
      });
      longitudinalToggle = speedLimitControllerToggle;
    } else if (param == "SLCConfirmation") {
      slcConfirmationBtn = new FrogPilotButtonsControl(title, desc, {tr("Lower Limits"), tr("Higher Limits"), tr("None")}, true, false);
      QObject::connect(slcConfirmationBtn, &FrogPilotButtonsControl::buttonClicked, [=](int id) {
        bool lowerEnabled = params.getBool("SLCConfirmationLower");
        bool higherEnabled = params.getBool("SLCConfirmationHigher");

        if (id == 0) {
          params.putBool("SLCConfirmationLower", !lowerEnabled);
          slcConfirmationBtn->setCheckedButton(0, !lowerEnabled);
          slcConfirmationBtn->setCheckedButton(2, false);

          if (lowerEnabled & !higherEnabled) {
            slcConfirmationBtn->setCheckedButton(2, true);
          }
        } else if (id == 1) {
          params.putBool("SLCConfirmationHigher", !higherEnabled);
          slcConfirmationBtn->setCheckedButton(1, !higherEnabled);
          slcConfirmationBtn->setCheckedButton(2, false);

          if (higherEnabled & !lowerEnabled) {
            slcConfirmationBtn->setCheckedButton(2, true);
          }
        } else {
          params.putBool("SLCConfirmationLower", false);
          params.putBool("SLCConfirmationHigher", false);
          slcConfirmationBtn->setCheckedButton(0, false);
          slcConfirmationBtn->setCheckedButton(1, false);
          slcConfirmationBtn->setCheckedButton(2, true);
        }
      });
      longitudinalToggle = slcConfirmationBtn;
    } else if (param == "SLCFallback") {
      std::vector<QString> fallbackOptions{tr("Set Speed"), tr("Experimental Mode"), tr("Previous Limit")};
      ButtonParamControl *fallbackSelection = new ButtonParamControl(param, title, desc, icon, fallbackOptions);
      longitudinalToggle = fallbackSelection;
    } else if (param == "SLCOverride") {
      std::vector<QString> overrideOptions{tr("None"), tr("Set With Gas Pedal"), tr("Max Set Speed")};
      ButtonParamControl *overrideSelection = new ButtonParamControl(param, title, desc, icon, overrideOptions);
      longitudinalToggle = overrideSelection;
    } else if (param == "SLCPriority") {
      ButtonControl *slcPriorityButton = new ButtonControl(title, tr("SELECT"), desc);
      QStringList primaryPriorities = {tr("None"), tr("Dashboard"), tr("Navigation"), tr("Offline Maps"), tr("Highest"), tr("Lowest")};
      QStringList secondaryTertiaryPriorities = {tr("None"), tr("Dashboard"), tr("Navigation"), tr("Offline Maps")};
      QStringList priorityPrompts = {tr("Select your primary priority"), tr("Select your secondary priority"), tr("Select your tertiary priority")};

      QObject::connect(slcPriorityButton, &ButtonControl::clicked, [=]() {
        QStringList selectedPriorities;

        for (int i = 1; i <= 3; ++i) {
          QStringList currentPriorities = (i == 1) ? primaryPriorities : secondaryTertiaryPriorities;
          QStringList prioritiesToDisplay = currentPriorities;

          for (QString selectedPriority : qAsConst(selectedPriorities)) {
            prioritiesToDisplay.removeAll(selectedPriority);
          }

          if (!hasDashSpeedLimits) {
            prioritiesToDisplay.removeAll(tr("Dashboard"));
          }

          if (prioritiesToDisplay.size() == 1 && prioritiesToDisplay.contains(tr("None"))) {
            break;
          }

          QString priorityKey = QString("SLCPriority%1").arg(i);
          QString selection = MultiOptionDialog::getSelection(priorityPrompts[i - 1], prioritiesToDisplay, "", this);

          if (selection.isEmpty()) {
            break;
          }

          params.putNonBlocking(priorityKey.toStdString(), selection.toStdString());
          selectedPriorities.append(selection);

          if (selection == tr("None")) {
            for (int j = i + 1; j <= 3; ++j) {
              QString priorityKeyNext = QString("SLCPriority%1").arg(j);
              params.putNonBlocking(priorityKeyNext.toStdString(), "None");
            }
            break;
          }

          if (selection == tr("Lowest") || selection == tr("Highest")) {
            break;
          }

          updateFrogPilotToggles();
        }

        selectedPriorities.removeAll(tr("None"));
        slcPriorityButton->setValue(selectedPriorities.join(", "));
      });

      QStringList initialPriorities;
      for (int i = 1; i <= 3; ++i) {
        QString priorityKey = QString("SLCPriority%1").arg(i);
        QString priority = QString::fromStdString(params.get(priorityKey.toStdString()));

        if (!priority.isEmpty() && primaryPriorities.contains(priority) && priority != tr("None")) {
          initialPriorities.append(priority);
        }
      }
      slcPriorityButton->setValue(initialPriorities.join(", "));
      longitudinalToggle = slcPriorityButton;
    } else if (param == "SLCOffsets") {
      ButtonControl *manageSLCOffsetsBtn = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(manageSLCOffsetsBtn, &ButtonControl::clicked, [this]() {
        openSubParentToggle();
        showToggles(speedLimitControllerOffsetsKeys);
      });
      longitudinalToggle = reinterpret_cast<AbstractControl*>(manageSLCOffsetsBtn);
    } else if (param == "SLCQOL") {
      ButtonControl *manageSLCQOLBtn = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(manageSLCQOLBtn, &ButtonControl::clicked, [this]() {
        openSubParentToggle();
        std::set<QString> modifiedSpeedLimitControllerQOLKeys = speedLimitControllerQOLKeys;

        if (hasPCMCruise) {
          modifiedSpeedLimitControllerQOLKeys.erase("SetSpeedLimit");
        }

        if (!isToyota) {
          modifiedSpeedLimitControllerQOLKeys.erase("ForceMPHDashboard");
        }

        showToggles(modifiedSpeedLimitControllerQOLKeys);
      });
      longitudinalToggle = reinterpret_cast<AbstractControl*>(manageSLCQOLBtn);
    } else if (param == "SLCLookaheadHigher" || param == "SLCLookaheadLower") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 60, tr(" seconds"));
    } else if (param == "Offset1" || param == "Offset2" || param == "Offset3" || param == "Offset4") {
      longitudinalToggle = new FrogPilotParamValueControl(param, title, desc, icon, -99, 99, tr("mph"));

    } else {
      longitudinalToggle = new ParamControl(param, title, desc, icon);
    }

    addItem(longitudinalToggle);
    toggles[param] = longitudinalToggle;

    makeConnections(longitudinalToggle);

    if (FrogPilotParamManageControl *frogPilotManageToggle = qobject_cast<FrogPilotParamManageControl*>(longitudinalToggle)) {
      QObject::connect(frogPilotManageToggle, &FrogPilotParamManageControl::manageButtonClicked, this, &FrogPilotLongitudinalPanel::openParentToggle);
    }

    QObject::connect(longitudinalToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  QObject::connect(static_cast<ToggleControl*>(toggles["ExperimentalModeViaLKAS"]), &ToggleControl::toggleFlipped, [this](bool state) {
    if (state && params.getBool("AlwaysOnLateralLKAS")) {
      params.putBoolNonBlocking("AlwaysOnLateralLKAS", false);
    }
  });

  FrogPilotParamValueControl *trafficFollowToggle = static_cast<FrogPilotParamValueControl*>(toggles["TrafficFollow"]);
  FrogPilotParamValueControl *trafficAccelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["TrafficJerkAcceleration"]);
  FrogPilotParamValueControl *trafficDecelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["TrafficJerkDeceleration"]);
  FrogPilotParamValueControl *trafficDangerToggle = static_cast<FrogPilotParamValueControl*>(toggles["TrafficJerkDanger"]);
  FrogPilotParamValueControl *trafficSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["TrafficJerkSpeed"]);
  FrogPilotParamValueControl *trafficSpeedDecreaseToggle = static_cast<FrogPilotParamValueControl*>(toggles["TrafficJerkSpeedDecrease"]);
  FrogPilotButtonsControl *trafficResetButton = static_cast<FrogPilotButtonsControl*>(toggles["ResetTrafficPersonality"]);
  QObject::connect(trafficResetButton, &FrogPilotButtonsControl::buttonClicked, this, [=]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to completely reset your settings for 'Traffic Mode'?"), this)) {
      params.putFloat("TrafficFollow", 0.5);
      params.putFloat("TrafficJerkAcceleration", 50);
      params.putFloat("TrafficJerkDeceleration", 50);
      params.putFloat("TrafficJerkDanger", 100);
      params.putFloat("TrafficJerkSpeed", 50);
      params.putFloat("TrafficJerkSpeedDecrease", 50);
      trafficFollowToggle->refresh();
      trafficAccelerationToggle->refresh();
      trafficDecelerationToggle->refresh();
      trafficDangerToggle->refresh();
      trafficSpeedToggle->refresh();
      trafficSpeedDecreaseToggle->refresh();
      updateFrogPilotToggles();
    }
  });

  FrogPilotParamValueControl *aggressiveFollowToggle = static_cast<FrogPilotParamValueControl*>(toggles["AggressiveFollow"]);
  FrogPilotParamValueControl *aggressiveAccelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["AggressiveJerkAcceleration"]);
  FrogPilotParamValueControl *aggressiveDecelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["AggressiveJerkDeceleration"]);
  FrogPilotParamValueControl *aggressiveDangerToggle = static_cast<FrogPilotParamValueControl*>(toggles["AggressiveJerkDanger"]);
  FrogPilotParamValueControl *aggressiveSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["AggressiveJerkSpeed"]);
  FrogPilotParamValueControl *aggressiveSpeedDecreaseToggle = static_cast<FrogPilotParamValueControl*>(toggles["AggressiveJerkSpeedDecrease"]);
  FrogPilotButtonsControl *aggressiveResetButton = static_cast<FrogPilotButtonsControl*>(toggles["ResetAggressivePersonality"]);
  QObject::connect(aggressiveResetButton, &FrogPilotButtonsControl::buttonClicked, this, [=]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to completely reset your settings for the 'Aggressive' personality?"), this)) {
      params.putFloat("AggressiveFollow", 1.25);
      params.putFloat("AggressiveJerkAcceleration", 50);
      params.putFloat("AggressiveJerkDeceleration", 50);
      params.putFloat("AggressiveJerkDanger", 100);
      params.putFloat("AggressiveJerkSpeed", 50);
      params.putFloat("AggressiveJerkSpeedDecrease", 50);
      aggressiveFollowToggle->refresh();
      aggressiveAccelerationToggle->refresh();
      aggressiveDecelerationToggle->refresh();
      aggressiveDangerToggle->refresh();
      aggressiveSpeedToggle->refresh();
      aggressiveSpeedDecreaseToggle->refresh();
      updateFrogPilotToggles();
    }
  });

  FrogPilotParamValueControl *standardFollowToggle = static_cast<FrogPilotParamValueControl*>(toggles["StandardFollow"]);
  FrogPilotParamValueControl *standardAccelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["StandardJerkAcceleration"]);
  FrogPilotParamValueControl *standardDecelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["StandardJerkDeceleration"]);
  FrogPilotParamValueControl *standardDangerToggle = static_cast<FrogPilotParamValueControl*>(toggles["StandardJerkDanger"]);
  FrogPilotParamValueControl *standardSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["StandardJerkSpeed"]);
  FrogPilotParamValueControl *standardSpeedDecreaseToggle = static_cast<FrogPilotParamValueControl*>(toggles["StandardJerkSpeedDecrease"]);
  FrogPilotButtonsControl *standardResetButton = static_cast<FrogPilotButtonsControl*>(toggles["ResetStandardPersonality"]);
  QObject::connect(standardResetButton, &FrogPilotButtonsControl::buttonClicked, this, [=]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to completely reset your settings for the 'Standard' personality?"), this)) {
      params.putFloat("StandardFollow", 1.45);
      params.putFloat("StandardJerkAcceleration", 100);
      params.putFloat("StandardJerkDeceleration", 100);
      params.putFloat("StandardJerkDanger", 100);
      params.putFloat("StandardJerkSpeed", 100);
      params.putFloat("StandardJerkSpeedDecrease", 100);
      standardFollowToggle->refresh();
      standardAccelerationToggle->refresh();
      standardDecelerationToggle->refresh();
      standardDangerToggle->refresh();
      standardSpeedToggle->refresh();
      standardSpeedDecreaseToggle->refresh();
      updateFrogPilotToggles();
    }
  });

  FrogPilotParamValueControl *relaxedFollowToggle = static_cast<FrogPilotParamValueControl*>(toggles["RelaxedFollow"]);
  FrogPilotParamValueControl *relaxedAccelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["RelaxedJerkAcceleration"]);
  FrogPilotParamValueControl *relaxedDecelerationToggle = static_cast<FrogPilotParamValueControl*>(toggles["RelaxedJerkDeceleration"]);
  FrogPilotParamValueControl *relaxedDangerToggle = static_cast<FrogPilotParamValueControl*>(toggles["RelaxedJerkDanger"]);
  FrogPilotParamValueControl *relaxedSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["RelaxedJerkSpeed"]);
  FrogPilotParamValueControl *relaxedSpeedDecreaseToggle = static_cast<FrogPilotParamValueControl*>(toggles["RelaxedJerkSpeedDecrease"]);
  FrogPilotButtonsControl *relaxedResetButton = static_cast<FrogPilotButtonsControl*>(toggles["ResetRelaxedPersonality"]);
  QObject::connect(relaxedResetButton, &FrogPilotButtonsControl::buttonClicked, this, [=]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to completely reset your settings for the 'Relaxed' personality?"), this)) {
      params.putFloat("RelaxedFollow", 1.75);
      params.putFloat("RelaxedJerkAcceleration", 100);
      params.putFloat("RelaxedJerkDeceleration", 100);
      params.putFloat("RelaxedJerkDanger", 100);
      params.putFloat("RelaxedJerkSpeed", 100);
      params.putFloat("RelaxedJerkSpeedDecrease", 100);
      relaxedFollowToggle->refresh();
      relaxedAccelerationToggle->refresh();
      relaxedDecelerationToggle->refresh();
      relaxedDangerToggle->refresh();
      relaxedSpeedToggle->refresh();
      relaxedSpeedDecreaseToggle->refresh();
      updateFrogPilotToggles();
    }
  });

  QObject::connect(parent, &FrogPilotSettingsWindow::closeParentToggle, this, &FrogPilotLongitudinalPanel::hideToggles);
  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubParentToggle, this, &FrogPilotLongitudinalPanel::hideSubToggles);
  QObject::connect(parent, &FrogPilotSettingsWindow::updateCarToggles, this, &FrogPilotLongitudinalPanel::updateCarToggles);
  QObject::connect(parent, &FrogPilotSettingsWindow::updateMetric, this, &FrogPilotLongitudinalPanel::updateMetric);

  updateMetric();
}

void FrogPilotLongitudinalPanel::showEvent(QShowEvent *event) {
  customizationLevel = parent->customizationLevel;

  toggles["ConditionalExperimental"]->setVisible(customizationLevel != 0);
  toggles["CurveSpeedControl"]->setVisible(customizationLevel != 0);
  toggles["CustomPersonalities"]->setVisible(customizationLevel == 2);
  toggles["ExperimentalModeActivation"]->setVisible(customizationLevel != 0);
  toggles["QOLLongitudinal"]->setVisible(customizationLevel != 0);
}

void FrogPilotLongitudinalPanel::updateCarToggles() {
  hasDashSpeedLimits = parent->hasDashSpeedLimits;
  hasPCMCruise = parent->hasPCMCruise;
  isGM = parent->isGM;
  isHKGCanFd = parent->isHKGCanFd;
  isSubaru = parent->isSubaru;
  isToyota = parent->isToyota;

  hideToggles();
}

void FrogPilotLongitudinalPanel::updateMetric() {
  bool previousIsMetric = isMetric;
  isMetric = params.getBool("IsMetric");

  if (isMetric != previousIsMetric) {
    double distanceConversion = isMetric ? FOOT_TO_METER : METER_TO_FOOT;
    double speedConversion = isMetric ? MILE_TO_KM : KM_TO_MILE;

    params.putFloatNonBlocking("IncreasedStoppedDistance", params.getFloat("IncreasedStoppedDistance") * distanceConversion);

    params.putFloatNonBlocking("CESignalSpeed", params.getFloat("CESignalSpeed") * speedConversion);
    params.putFloatNonBlocking("CESpeed", params.getFloat("CESpeed") * speedConversion);
    params.putFloatNonBlocking("CESpeedLead", params.getFloat("CESpeedLead") * speedConversion);
    params.putFloatNonBlocking("CustomCruise", params.getFloat("CustomCruise") * speedConversion);
    params.putFloatNonBlocking("CustomCruiseLong", params.getFloat("CustomCruiseLong") * speedConversion);
    params.putFloatNonBlocking("Offset1", params.getFloat("Offset1") * speedConversion);
    params.putFloatNonBlocking("Offset2", params.getFloat("Offset2") * speedConversion);
    params.putFloatNonBlocking("Offset3", params.getFloat("Offset3") * speedConversion);
    params.putFloatNonBlocking("Offset4", params.getFloat("Offset4") * speedConversion);
    params.putFloatNonBlocking("SetSpeedOffset", params.getFloat("SetSpeedOffset") * speedConversion);
  }

  FrogPilotDualParamControl *ceSpeedToggle = reinterpret_cast<FrogPilotDualParamControl*>(toggles["CESpeed"]);
  FrogPilotParamValueButtonControl *ceSignal = reinterpret_cast<FrogPilotParamValueButtonControl*>(toggles["CESignalSpeed"]);
  FrogPilotParamValueControl *customCruiseToggle = static_cast<FrogPilotParamValueControl*>(toggles["CustomCruise"]);
  FrogPilotParamValueControl *customCruiseLongToggle = static_cast<FrogPilotParamValueControl*>(toggles["CustomCruiseLong"]);
  FrogPilotParamValueControl *offset1Toggle = static_cast<FrogPilotParamValueControl*>(toggles["Offset1"]);
  FrogPilotParamValueControl *offset2Toggle = static_cast<FrogPilotParamValueControl*>(toggles["Offset2"]);
  FrogPilotParamValueControl *offset3Toggle = static_cast<FrogPilotParamValueControl*>(toggles["Offset3"]);
  FrogPilotParamValueControl *offset4Toggle = static_cast<FrogPilotParamValueControl*>(toggles["Offset4"]);
  FrogPilotParamValueControl *increasedStoppedDistanceToggle = static_cast<FrogPilotParamValueControl*>(toggles["IncreasedStoppedDistance"]);
  FrogPilotParamValueControl *setSpeedOffsetToggle = static_cast<FrogPilotParamValueControl*>(toggles["SetSpeedOffset"]);

  if (isMetric) {
    offset1Toggle->setTitle(tr("Speed Limit Offset (0-34 kph)"));
    offset2Toggle->setTitle(tr("Speed Limit Offset (35-54 kph)"));
    offset3Toggle->setTitle(tr("Speed Limit Offset (55-64 kph)"));
    offset4Toggle->setTitle(tr("Speed Limit Offset (65-99 kph)"));

    offset1Toggle->setDescription(tr("Sets the speed limit offset for speeds between 0-34 kph."));
    offset2Toggle->setDescription(tr("Sets the speed limit offset for speeds between 35-54 kph."));
    offset3Toggle->setDescription(tr("Sets the speed limit offset for speeds between 55-64 kph."));
    offset4Toggle->setDescription(tr("Sets the speed limit offset for speeds between 65-99 kph."));

    ceSignal->updateControl(0, 150, tr("kph"));
    ceSpeedToggle->updateControl(0, 150, tr("kph"));
    customCruiseToggle->updateControl(1, 150, tr("kph"));
    customCruiseLongToggle->updateControl(1, 150, tr("kph"));
    offset1Toggle->updateControl(-99, 99, tr("kph"));
    offset2Toggle->updateControl(-99, 99, tr("kph"));
    offset3Toggle->updateControl(-99, 99, tr("kph"));
    offset4Toggle->updateControl(-99, 99, tr("kph"));
    setSpeedOffsetToggle->updateControl(0, 150, tr("kph"));

    increasedStoppedDistanceToggle->updateControl(0, 3, tr(" meters"));
  } else {
    offset1Toggle->setTitle(tr("Speed Limit Offset (0-34 mph)"));
    offset2Toggle->setTitle(tr("Speed Limit Offset (35-54 mph)"));
    offset3Toggle->setTitle(tr("Speed Limit Offset (55-64 mph)"));
    offset4Toggle->setTitle(tr("Speed Limit Offset (65-99 mph)"));

    offset1Toggle->setDescription(tr("Sets the speed limit offset for speeds between 0-34 mph."));
    offset2Toggle->setDescription(tr("Sets the speed limit offset for speeds between 35-54 mph."));
    offset3Toggle->setDescription(tr("Sets the speed limit offset for speeds between 55-64 mph."));
    offset4Toggle->setDescription(tr("Sets the speed limit offset for speeds between 65-99 mph."));

    ceSignal->updateControl(0, 99, tr("mph"));
    ceSpeedToggle->updateControl(0, 99, tr("mph"));
    customCruiseToggle->updateControl(1, 99, tr("mph"));
    customCruiseLongToggle->updateControl(1, 99, tr("mph"));
    offset1Toggle->updateControl(-99, 99, tr("mph"));
    offset2Toggle->updateControl(-99, 99, tr("mph"));
    offset3Toggle->updateControl(-99, 99, tr("mph"));
    offset4Toggle->updateControl(-99, 99, tr("mph"));
    setSpeedOffsetToggle->updateControl(0, 99, tr("mph"));

    increasedStoppedDistanceToggle->updateControl(0, 10, tr(" feet"));
  }
}

void FrogPilotLongitudinalPanel::showToggles(const std::set<QString> &keys) {
  setUpdatesEnabled(false);

  for (auto &[key, toggle] : toggles) {
    toggle->setVisible(keys.find(key) != keys.end());
  }

  setUpdatesEnabled(true);
  update();
}

void FrogPilotLongitudinalPanel::hideToggles() {
  setUpdatesEnabled(false);

  customPersonalityOpen = false;
  slcOpen = false;

  for (auto &[key, toggle] : toggles) {
    bool subToggles = aggressivePersonalityKeys.find(key) != aggressivePersonalityKeys.end() ||
                      conditionalExperimentalKeys.find(key) != conditionalExperimentalKeys.end() ||
                      curveSpeedKeys.find(key) != curveSpeedKeys.end() ||
                      customDrivingPersonalityKeys.find(key) != customDrivingPersonalityKeys.end() ||
                      experimentalModeActivationKeys.find(key) != experimentalModeActivationKeys.end() ||
                      longitudinalTuneKeys.find(key) != longitudinalTuneKeys.end() ||
                      qolKeys.find(key) != qolKeys.end() ||
                      relaxedPersonalityKeys.find(key) != relaxedPersonalityKeys.end() ||
                      speedLimitControllerKeys.find(key) != speedLimitControllerKeys.end() ||
                      speedLimitControllerOffsetsKeys.find(key) != speedLimitControllerOffsetsKeys.end() ||
                      speedLimitControllerQOLKeys.find(key) != speedLimitControllerQOLKeys.end() ||
                      standardPersonalityKeys.find(key) != standardPersonalityKeys.end() ||
                      trafficPersonalityKeys.find(key) != trafficPersonalityKeys.end();

    toggle->setVisible(!subToggles);

    toggles["ConditionalExperimental"]->setVisible(customizationLevel != 0);
    toggles["CurveSpeedControl"]->setVisible(customizationLevel != 0);
    toggles["CustomPersonalities"]->setVisible(customizationLevel == 2);
    toggles["ExperimentalModeActivation"]->setVisible(customizationLevel != 0);
    toggles["QOLLongitudinal"]->setVisible(customizationLevel != 0);
  }

  setUpdatesEnabled(true);
  update();
}

void FrogPilotLongitudinalPanel::hideSubToggles() {
  if (customPersonalityOpen) {
    customPersonalityOpen = false;
    showToggles(customDrivingPersonalityKeys);
  } else if (slcOpen) {
    std::set<QString> modifiedSpeedLimitControllerKeys = speedLimitControllerKeys;

    if (customizationLevel == 0) {
      modifiedSpeedLimitControllerKeys.erase("SLCFallback");
      modifiedSpeedLimitControllerKeys.erase("SLCOverride");
      modifiedSpeedLimitControllerKeys.erase("SLCPriority");
      modifiedSpeedLimitControllerKeys.erase("SLCQOL");
    }

    showToggles(modifiedSpeedLimitControllerKeys);
  }
}
