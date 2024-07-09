package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Buttons {

    private int incrementer;
    private final ArrayList<Boolean> LIST;
    private int held_incrementer;
    private final ArrayList<double[]> HELD_LIST;

    public Buttons(){
        this.held_incrementer = 0;
        this.incrementer = 0;
        this.LIST = new ArrayList<>();
        this.HELD_LIST = new ArrayList<>();
    }

    public boolean ifPressed(boolean button){ // down = do you want to detect when the button is pressed down or up

        boolean output;

        if (this.LIST.size() == this.incrementer){ // if there isn't a value for the button then make one.
            this.LIST.add(false);
        }

        boolean b_was = this.LIST.get(this.incrementer); // last value of the button

        output = (button != b_was && button); // if the button isn't in the same state it was and (if down is true) the button is now down then the button was pressed.

        this.LIST.set(this.incrementer, button);

        this.incrementer++;
        return output;

    }

    public boolean time_held(boolean button, double currentTime, double time_held){ // down = do you want to detect when the button is pressed down or up

        boolean output;

        if (this.HELD_LIST.size() == this.held_incrementer || (this.HELD_LIST.get(this.held_incrementer)[1] == 0 && button)){ // if there isn't a value for the button then make one.
            this.HELD_LIST.add(new double[]{currentTime, 0}); // time you started pressing, how long it has been held
        }

        double b_was = this.HELD_LIST.get(this.held_incrementer)[1]; // last value of the button
        output = (b_was > time_held);

        if (button) {

            this.HELD_LIST.set(this.held_incrementer, new double[] {this.HELD_LIST.get(this.held_incrementer)[0], currentTime - this.HELD_LIST.get(this.held_incrementer)[0]});

        } else {

            this.HELD_LIST.set(this.held_incrementer, new double[] {this.HELD_LIST.get(this.held_incrementer)[0], 0});

        }

        this.held_incrementer++;
        return output;

    }

    public void reset(){
        this.incrementer = 0;
        this.held_incrementer = 0;
    }
    
}
