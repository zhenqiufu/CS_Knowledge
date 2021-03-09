
Using [Google C Style] (https://google.github.io/styleguide/cppguide.html) in Sublime Text

In this paper, I will introduce how to use Sublime Text to write C code with Google C Style.

install step:

1. ctrl+shift+p and input "install" to install Package Control.
2. ctrl+shift+p and input "install" to chose "Package Control: Install Package"
3. input "SublimeAStyleFormatter ", and white a second, it will complete install.

How to use

1. write a piece of C code;
2. right click and choose "AStyleFormater->Format", it can be formatted.

if you want to modify something in your setting, you can open "Preferences->Package Settings->SublimeAStyleFormatter->Settings - Default" and "Settings - User" and then copy the setting you want to change from "Settings - Default" to "Settings - User".

SublimeAStyleFormatter use Google C Style

1. after copy the default setting to user setting, change the 28th line "style" from "null" to "google".


2. using the "ctrl+s" to format. change the 8th line "autoformat_on_save" from "false" to "true".


finish!

But it is not very well. AStyleFormater can not delete the multi space line.

I find the clang format is better than AStyleFormater.

1. install clang format pluggin using Package Control.
2. sudo apt-get install clang-format
 
finish!

