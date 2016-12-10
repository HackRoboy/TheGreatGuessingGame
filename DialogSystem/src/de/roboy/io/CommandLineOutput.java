package de.roboy.io;

import java.util.List;

import de.roboy.dialog.action.Action;
import de.roboy.dialog.action.SpeechAction;

public class CommandLineOutput implements OutputDevice{

	@Override
	public void act(List<Action> actions) {
		for(Action a : actions){
			if(a instanceof SpeechAction){
				System.out.println(((SpeechAction) a).getText());
			}
		}
	}
	public void act(String output){
		System.out.println(output);
	}

}
