package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Buttons {

    private int incrementer;
    private final ArrayList<Boolean> LIST;

    public Buttons(){
        this.incrementer = 0;
        this.LIST = new ArrayList<>();
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

    public void reset(){
        this.incrementer = 0;
    }
    
}
