package de.roboy.dialog.personality;

import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import de.roboy.dialog.action.Action;
import de.roboy.dialog.action.ShutDownAction;
import de.roboy.dialog.action.SpeechAction;
import de.roboy.io.ClassificationInput;
import de.roboy.io.Input;
import de.roboy.io.SentenceInput;
import de.roboy.linguistics.sentenceanalysis.Interpretation;
import de.roboy.util.GsonHelper;

public class GuessingGamePersonality implements Personality {

	private enum GuessingGameState {
		WELCOME, REGISTER_GROUP_A, REGISTER_GROUP_B, REGISTER_TERM, REGISTER_STOP_WORDS, GROUP_A_SENTENCE_INPUT, GROUP_B_SENTENCE_INPUT, TELL_RESULTS
	}

	private GuessingGameState state = GuessingGameState.WELCOME;
	private static String groupA, groupB;
	private static String term = "";
	private static List<String> stopWords;

	@Override
	public List<Action> answer(Interpretation input) {
		List<Action> result = new ArrayList<Action>();
		List<Input> listInput = GsonHelper.decode(input.sentence);
		SentenceInput sentence = null;
		ClassificationInput classification = null;
		if (listInput != null) {
			for (Input i : listInput) {
				if (i.getClass() == SentenceInput.class) {
					// TODO multiple Inputs?
					sentence = (SentenceInput) i;
				} else {
					classification = (ClassificationInput) i;
				}
			}
		}
		switch (state) {
		case WELCOME:
			result.add(new SpeechAction("Welcome to the Guessing Game with Roboy"));
			state = GuessingGameState.REGISTER_GROUP_A;
			// return result;
		case REGISTER_GROUP_A:
			result.add(new SpeechAction("Group 1, please tell me the name of your group."));
			state = GuessingGameState.REGISTER_GROUP_B;
			return result;
		case REGISTER_GROUP_B:
			// save and greet groupA
			System.out.println(input.sentence);
			//TODO
			if (listInput == null) {
				if (groupA.equals("")) {
					for (Input in : listInput) {
						if (in.getClass() == SentenceInput.class) {
							if (((SentenceInput) in).getInput().length() <= 30) {
								groupA = input.sentence;
								result.add(new SpeechAction("Group 1 will now be called " + groupA));
							} else {
								result.add(new SpeechAction(
										"Group 1, the name is too long. Please choose a shorter one."));
								state = GuessingGameState.REGISTER_GROUP_B;
								return result;
							}
						}
					}
				}
			} else {
				result.add(new SpeechAction("Invalid Input.\nGroup 1, please choose a groupname."));
				state = GuessingGameState.REGISTER_GROUP_B;
				return result;
			}

			// ask groupB
			result.add(new SpeechAction("Group 2, please tell me the name of your group."));
			state = GuessingGameState.REGISTER_TERM;
			return result;

		case REGISTER_TERM:
			// save and greet groupB
			if (listInput != null) {
				if (groupB.equals("")) {
					for (Input in : listInput) {
						if (in.getClass() == SentenceInput.class) {
							if (((SentenceInput) in).getInput().length() <= 30) {
								groupB = input.sentence;
								result.add(new SpeechAction("Group 2 will now be called " + groupB));
							} else {
								result.add(new SpeechAction(
										"Group 2, the name is too long. Please choose a shorter one."));
								state = GuessingGameState.REGISTER_TERM;
								return result;
							}
						}
					}
				}
			} else {
				result.add(new SpeechAction("Invalid Input.\nGroup 2, please choose a groupname."));
				state = GuessingGameState.REGISTER_TERM;
				return result;
			}

			// TODO

		default:
			List<Action> lastwords = new ArrayList<Action>();
			lastwords.add(new SpeechAction("I am lost, bye."));
			result.add(new ShutDownAction(lastwords));
			return result;
		}
	}

	private void filterSpamWords() {
		// TODO take out words like 'and', 'or', .. of the sentence and return
		// the filtered sentence
		// filter with POS: UH, ..
	}

}
