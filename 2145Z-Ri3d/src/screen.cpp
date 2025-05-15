#include "liblvgl/misc/lv_anim.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "gif-pros/gifclass.hpp"
#include "liblvgl/lvgl.h"
#include "pros/apix.h"
#include "screen.hpp"
#include "autons.hpp"
#include <vector>
#include "api.h"
#include <cstdlib> 
#include <cmath>
#include <ctime> 
#include <fstream>
#include <stdio.h>

// ** @file screen.cpp
// ** @brief This file contains the functions for the robot's brain screen.
// ** @details This includes the functions for the calibration screen, the autonomous selector screen, and the gif display.
// ** @author Ansh Rao - 2145Z, credit to Baker - 2145Z for the inspiration - https://github.com/bbruce1/2145ZBotCode

// DEFAULT VALUE
// DEFAULT VALUE
// DEFAULT VALUE
// DEFAULT VALUE

int selectedAuton = 0; // will be overridden by sd card if exists

// DEFAULT VALUE
// DEFAULT VALUE
// DEFAULT VALUE
// DEFAULT VALUE
// DEFAULT VALUE



// Function to make a color lighter or darker
lv_color_t color_shade(lv_color_t base_color, int percentage) {
    // Separate the color into its RGB components
    int red = lv_color_to32(base_color) >> 16 & 0xFF;
    int green = lv_color_to32(base_color) >> 8 & 0xFF;
    int blue = lv_color_to32(base_color) & 0xFF;

    // Calculate the adjustment based on the percentage
    float adjust = (100 + percentage) / 100.0f;

    // Adjust and clamp each color component
    red = std::min(std::max(0, static_cast<int>(red * adjust)), 255);
    green = std::min(std::max(0, static_cast<int>(green * adjust)), 255);
    blue = std::min(std::max(0, static_cast<int>(blue * adjust)), 255);

    // Recombine the color components and cast to uint8_t to avoid narrowing conversion
    return LV_COLOR_MAKE(static_cast<uint8_t>(red), static_cast<uint8_t>(green), static_cast<uint8_t>(blue));
}


std::string pickRandomString(const std::string strings[], int size) {
    double voltage = pros::battery::get_voltage();
    double capacity = pros::battery::get_capacity();
    double current = pros::battery::get_current();
    double temperature = pros::battery::get_temperature();

    double seedValue = pow((voltage * capacity * current * temperature), 3.0);

    unsigned int seed = static_cast<unsigned int>(fmod(seedValue, UINT_MAX));

    // Seed the random number generator
    srand(seed);    
    
    // Check if the size is not zero to avoid division by zero
    if (size <= 0) {
        return ""; // Or handle the error as appropriate
    }

    // Generate a random index between 0 and size - 1
    int index = rand() % size;

    // Return the string at the random index
    return strings[index];
}

bool chassisCalibrated = false;

// these are the gifs that will display randomly for the calibration screen
std::string calibrationGifs[] = {
                "/usd/jq-griddy.gif", 
                "/usd/dog-angry.gif", 
                "/usd/carnage-venom.gif", 
                "/usd/beastboy.gif", 
            };

// these are the gifs that will display randomly during the match
std::string matchGifs[] = {
                "/usd/getreal.gif", 
            };

int matchGifsSize = sizeof(matchGifs) / sizeof(matchGifs[0]);
int calibrationGifsSize = sizeof(calibrationGifs) / sizeof(calibrationGifs[0]);

void calibrationScreen() {
    // Get a random gif string
    std::string randomGif = pickRandomString(calibrationGifs, calibrationGifsSize);

    // Create a new Gif object with the random string as a C-style string
    Gif* caliGif = new Gif(randomGif.c_str(), lv_scr_act());

    while (!chassisCalibrated) {
        pros::delay(25);
    }

    delete caliGif; // Properly delete the dynamically allocated Gif object
}


void calibrationScreenInit() {
    pros::Task calibrationScreenTask(calibrationScreen); 
}

lv_obj_t *roller;
lv_obj_t *confirmBtn;
lv_obj_t *confirmLabel;
Gif *textGif;

int readSelectedAutonFromFile() {
    FILE* usd_file_read = fopen("/usd/auton_selection.txt", "r");

    if (usd_file_read == NULL) {
        // File does not exist or cannot be opened, default to 0
        return selectedAuton;  // Return the default value
    }

    // If the file exists, read the integer value
    if (fscanf(usd_file_read, "%d", &selectedAuton) != 1) {
        // If reading fails or the data is not an integer, default to 0
        selectedAuton = selectedAuton;
    }

    fclose(usd_file_read);  // Always close the file
    return selectedAuton;  // Return the read or default value
}

void setSelectorOptions(lv_obj_t *roller, size_t autonCount, AutonFunction autonFunctions[]) {
    // Calculate the length needed for the options string
    size_t optionsLength = 0;
    for (size_t i = 0; i < autonCount; ++i) {
        optionsLength += strlen(autonFunctions[i].name) + 1; // +1 for the newline character
    }

    // Allocate memory for the options string
    char *options = (char *)malloc(optionsLength + 1); // +1 for the null terminator
    if (options == NULL) {
        // Handle memory allocation error
        return;
    }

    // Concatenate all auton names into the options string
    char *currentPosition = options;
    for (size_t i = 0; i < autonCount; ++i) {
        strcpy(currentPosition, autonFunctions[i].name);
        currentPosition += strlen(autonFunctions[i].name);
        *currentPosition = '\n'; // Add newline character
        currentPosition++;
    }
    *(currentPosition - 1) = '\0'; // Replace the last newline character with a null terminator

    // Set the roller options
    lv_roller_set_options(roller, options, LV_ROLLER_MODE_NORMAL);

    if (selectedAuton < autonCount) {
        lv_roller_set_selected(roller, selectedAuton, LV_ANIM_OFF);
    }

    // Free the memory for the options string
    free(options);
}

void autonList(lv_obj_t *scr, lv_color_t color_scheme, AutonFunction autonFunctions[], size_t autonCount) {
    // Create a style for the roller background
    static lv_style_t style_bg;
    lv_style_init(&style_bg);
    lv_style_set_bg_color(&style_bg, lv_color_black());
    lv_style_set_text_color(&style_bg, color_scheme);

    // Create a style for the roller's selected item
    static lv_style_t style_sel;
    lv_style_init(&style_sel);
    lv_style_set_bg_color(&style_sel, lv_color_black());
    lv_style_set_border_color(&style_sel, lv_color_white());
    lv_style_set_text_color(&style_sel, color_scheme);

    // Create the roller
    roller = lv_roller_create(scr);
    setSelectorOptions(roller, autonCount, autonFunctions);

    // Apply the styles to the roller
    lv_obj_add_style(roller, &style_bg, LV_PART_MAIN);
    lv_obj_add_style(roller, &style_sel, LV_PART_SELECTED);

    // Set the size and alignment of the roller
    lv_obj_set_size(roller, lv_obj_get_width(scr) / 3, lv_obj_get_height(scr) - 20);
    lv_obj_align(roller, LV_ALIGN_LEFT_MID, 10, 0);
}

// Function to be called when the confirm button is pressed
static lv_res_t confirm_btn_action(lv_obj_t *btn) {    
    // Get the selected option index from the roller
    selectedAuton = lv_roller_get_selected(roller);
    FILE* usd_file_write = fopen("/usd/auton_selection.txt", "w");
    fprintf(usd_file_write, "%d\n", selectedAuton); 
    fclose(usd_file_write);
    

    // Create a new screen
    lv_obj_t *new_scr = lv_obj_create(NULL);


    // Load the new screen
    lv_scr_load(new_scr);

    // Ensure mainScreenGif is not null and is properly initialized

    return LV_RES_OK;
}

lv_obj_t* confirmButton(lv_obj_t *scr, lv_color_t color_scheme) { 
    // Create a style for the confirm button
    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, color_scheme);
    lv_style_set_text_color(&style_btn, lv_color_white());

    // Create the confirm button
    lv_obj_t *confirmBtn = lv_btn_create(scr);
    lv_obj_add_style(confirmBtn, &style_btn, LV_PART_MAIN);
    lv_obj_set_size(confirmBtn, 120, 50);
    lv_obj_align(confirmBtn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);

    // Add a label to the confirm button
    lv_obj_t *confirmLabel = lv_label_create(confirmBtn);
    lv_label_set_text(confirmLabel, "Confirm");

    return confirmBtn;
}

lv_obj_t* editPortsButton(lv_obj_t *scr, lv_color_t color_scheme) { 
    // Create a style for the button
    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, color_scheme); // Use the passed color
    lv_style_set_bg_grad_color(&style_btn, color_shade(color_scheme, -10)); // Make the gradient slightly darker
    lv_style_set_radius(&style_btn, 10); // Round the corners a bit
    lv_style_set_text_color(&style_btn, lv_color_white()); // White text on the button
    lv_style_set_border_color(&style_btn, lv_color_white()); // White border for cool effect
    lv_style_set_border_width(&style_btn, 2); // Set the border width
    lv_style_set_shadow_color(&style_btn, lv_color_hex(0xC0C0C0)); // Shadow effect for cool look
    lv_style_set_shadow_width(&style_btn, 5); // Set the shadow width
    lv_style_set_text_font(&style_btn, &lv_font_montserrat_20); // Set the font for the text

    // Create a style for the button when it's pressed
    static lv_style_t style_btn_pr;
    lv_style_init(&style_btn_pr);
    lv_style_set_bg_color(&style_btn_pr, color_shade(color_scheme, -30)); // Darker color for pressed state
    lv_style_set_bg_grad_color(&style_btn_pr, color_shade(color_scheme, -40)); // Even darker for gradient
    lv_style_set_text_color(&style_btn_pr, color_shade(color_scheme, 50)); // Lighter color for the text
    lv_style_set_border_color(&style_btn_pr, lv_color_white()); // Keep the border color
    lv_style_set_shadow_width(&style_btn_pr, 3); // Smaller shadow for a "pressed" effect

    // Create the button
    lv_obj_t *editBtn = lv_btn_create(scr);
    lv_obj_add_style(editBtn, &style_btn, LV_PART_MAIN); // Apply the released style to the button
    lv_obj_add_style(editBtn, &style_btn_pr, LV_PART_MAIN | LV_STATE_PRESSED); // Apply the pressed style to the button
    lv_obj_set_size(editBtn, 120, 50); // Set the button size
    lv_obj_align(editBtn, LV_ALIGN_BOTTOM_RIGHT, -140, -10); // Align to the bottom-right with some margin

    // Add a label to the button
    lv_obj_t *editLabel = lv_label_create(editBtn);
    lv_label_set_text(editLabel, "Edit Ports");

    return editBtn;
}

Gif* gifContainer(lv_obj_t *scr) {
    // Create an outer container for positioning
    lv_obj_t *outer_gif_cont = lv_obj_create(scr);
    lv_obj_set_size(outer_gif_cont, LV_HOR_RES / 1.6, LV_VER_RES / 1.55);
    lv_obj_align(outer_gif_cont, LV_ALIGN_TOP_RIGHT, -10, 10);

    // Create and place the GIF inside the container
    textGif = new Gif("/usd/2145shake.gif", outer_gif_cont);
    textGif->resume(); // Start playing the GIF
    return textGif;
}

void autonSelectorScreenInit(AutonFunction autonFunctions[], size_t autonCount, lv_color_t color_scheme) {
    chassisCalibrated = true;
    readSelectedAutonFromFile();

    // Create a screen object
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_scr_load(scr);

    autonList(scr, color_scheme, autonFunctions, autonCount);
    confirmButton(scr, color_scheme);
}

void runSelectedAuton(AutonFunction autonFunctions[], size_t autonCount) {
    if (selectedAuton >= 0 && selectedAuton < autonCount) {
        autonFunctions[selectedAuton].function();
    }
}