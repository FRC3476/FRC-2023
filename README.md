# Training 11/26

Oh no! Some malicious people have made some commits and now we don't have buttons assigned for our robot anymore!

[Here](./src/main/java/frc/robot/Robot.java) is all we have left... it is now up to you to reassign the buttons. 
We will be running your code and testing your buttons.

These classes will be helpful.
```
frc.utility.Controller.XboxAxes
frc.utility.Controller.XboxButtons
```

Feel free to refer to the `main` branch if you're confused.

Note: If you use one of the Triggers from the `XboxAxes` class,
you must add a threshold value (how far the button must be pushed in order to trigger the desired command).
To do this, right click the variable name and click on "Find Usages", and in every instance of `xbox.getRisingEdge(XBOX_BUTTON_DO_THING)`, append your desired threshold value, like so: `xbox.getRisingEdge(XBOX_BUTTON_DO_THING, 0.1)`