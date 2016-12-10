package de.roboy.dialog.personality;

import java.util.ArrayList;
import java.util.List;

import de.roboy.dialog.action.Action;
import de.roboy.dialog.action.SpeechAction;
import de.roboy.linguistics.sentenceanalysis.Interpretation;

public class GuessingGamePersonality implements Personality{

	private enum GuessingGameState {WELCOME, REGISTER_GROUP_A, REGISTER_GROUP_B, REGISTER_TERM, REGISTER_STOP_WORDS, 
		GROUP_A_SENTENCE_INPUT, GROUP_B_SENTENCE_INPUT, TELL_RESULTS}
	private GuessingGameState state = GuessingGameState.WELCOME;
	
	@Override
	public List<Action> answer(Interpretation input) {
		List<Action> result = new ArrayList<Action>();
		switch (state){
		case WELCOME:
			result.add(new SpeechAction("Welcome to the Guessing Game with Roboy"));
			state = GuessingGameState.REGISTER_GROUP_A;
			return result;
		case REGISTER_GROUP_A:
			
			result.add(new SpeechAction("Group 1, please tell me the name of your group."));
			state = GuessingGameState.GROUP_B_SENTENCE_INPUT;
			return result;
			//TODO
		default:
			break;
		}
		
		return null;
	}

}
