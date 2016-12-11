package de.roboy.dialog.personality;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import de.roboy.dialog.action.Action;
import de.roboy.dialog.action.EmotionAction;
import de.roboy.dialog.action.ShutDownAction;
import de.roboy.dialog.action.SpeechAction;
import de.roboy.guessinggame.Association;
import de.roboy.io.ClassificationInput;
import de.roboy.io.Input;
import de.roboy.io.SentenceInput;
import de.roboy.linguistics.sentenceanalysis.Interpretation;
import de.roboy.util.GsonHelper;

public class GuessingGamePersonality implements Personality {

	private enum GuessingGameState {
		WELCOME, REGISTER_GROUP_A, REGISTER_GROUP_B, REGISTER_TERM, REGISTER_STOP_WORDS, FINALIZE_SETUP, GROUP_A_SENTENCE_INPUT, GROUP_B_SENTENCE_INPUT, TELL_RESULTS
	}

	private final static int GROUP_A = 0;
	private final static int GROUP_B = 1;
	private final static double POINT_BORDER = 6.0; // TODO set border
	private GuessingGameState state = GuessingGameState.WELCOME;
	private String groupA = "";
	private String groupB = "";
	private boolean groupAsTurnFirst = true;
	private boolean firstRound = true;
	private String term = "";
	private HashSet<String> stopWords = new HashSet<String>();
	private ClassificationInput classification;
	private SentenceInput sentence;
	private Association association; // at Index 0: Nouns; Index 1: Verbs; Index
										// 2: Adjectives
	private int pointsA = 0;
	private int pointsB = 0;
	private double probTeamA = 0;
	private double probTeamB = 0;

	@Override
	public List<Action> answer(Interpretation input) {
		List<Action> result = new ArrayList<Action>();
		/*
		 * TODO Emotion cycle; At the beginning of a state the input for the
		 * previous state gets processed and if the input is not valid, it
		 * repeats this state
		 */
		switch (state) {
		case WELCOME:
			// TODO more introduction to the game
			result.add(new SpeechAction("Welcome to the Guessing Game with Roboy. "));
			state = GuessingGameState.REGISTER_GROUP_A;
			// return result;
		case REGISTER_GROUP_A:
			// Ask groupA for groupname
			result.add(new SpeechAction("Group 1, please tell me the name of your group."));
			state = GuessingGameState.REGISTER_GROUP_B;
			return result;
		case REGISTER_GROUP_B:
			// save and greet groupA
			if (sentence != null) {
				if (groupA.equals("")) {
					if (sentence.getInput().length() <= 30) {
						groupA = sentence.getInput();
						result.add(new SpeechAction("Group 1 will now be called " + groupA));
					} else {
						result.add(new SpeechAction("Group 1, the name is too long. Please choose a shorter one."));
						state = GuessingGameState.REGISTER_GROUP_B;
						return result;
					}
				}
			} else {
				result.add(new SpeechAction("Invalid Input.\nGroup 1, please choose a groupname."));
				state = GuessingGameState.REGISTER_GROUP_B;
				return result;
			}

			// ask groupB for groupname
			result.add(new SpeechAction("Group 2, please tell me the name of your group."));
			state = GuessingGameState.REGISTER_TERM;
			return result;

		case REGISTER_TERM:
			// save and greet groupB
			if (sentence != null) {
				if (groupB.equals("")) {
					if (sentence.getInput().length() <= 30) {
						groupB = sentence.getInput();
						result.add(new SpeechAction("Group 2 will now be called " + groupB));
					} else {
						result.add(new SpeechAction("Group 2, the name is too long. Please choose a shorter one."));
						state = GuessingGameState.REGISTER_TERM;
						return result;
					}
				}
			} else {
				result.add(new SpeechAction("Invalid Input.\nGroup 2, please choose a groupname."));
				state = GuessingGameState.REGISTER_TERM;
				return result;
			}

			// ask if not first round if they want to continue playing,
			// tell current points
			if (!firstRound) {
				groupAsTurnFirst = !groupAsTurnFirst;
				result.add(new SpeechAction("Currently " + groupA + " has " + pointsA + " points. " + groupB + " has "
						+ pointsB + " points."));
				if (sentence != null) {
					if (sentence.getInput().toLowerCase().contains("yes")) {
						// play another round
						result.add(new SpeechAction(
								"Let's play another round. Please tell me the object yout want me to guess."));
						term = "";
						probTeamA = 0;
						probTeamB = 0;
						state = GuessingGameState.REGISTER_STOP_WORDS;
						return result;
					} else if (sentence.getInput().toLowerCase().contains("no")) {
						// go to TELL_RESULTS
						result.add(new SpeechAction("Thank you for playing with me. Did you enjoy it?"));
						state = GuessingGameState.TELL_RESULTS;
						return result;
					} else {
						// ask again
						result.add(new SpeechAction("I did not understand you. Do you want to play another round?"));
						state = GuessingGameState.REGISTER_TERM;
						return result;
					}
				}
			}

			// Ask for Term
			result.add(new SpeechAction("Please tell me the object yout want me to guess."));
			state = GuessingGameState.REGISTER_STOP_WORDS;
			return result;

		case REGISTER_STOP_WORDS:
			// once they got here, they won't get back to REGISTER_TERM
			firstRound = false;
			// check input for term
			if (sentence == null) {
				result.add(new SpeechAction("You did not say an object. You don't want to play with me."));
				result.add(new EmotionAction("sad"));
				result.add(new SpeechAction("Try again. Please tell me the object yout want me to guess."));
				state = GuessingGameState.REGISTER_STOP_WORDS;
				return result;
			}
			if (classification != null) {
				if (term.equals("")) {
					int iSeeIt;
					if (classification.getProbabilities().containsKey(sentence.getInput())) {
						term = sentence.getInput();
						// call python script to get associations and process
						// them
						association = getAssociations(term);
						if (association == null) {
							result.add(new SpeechAction(
									"I'm sorry but I don't know what " + sentence.getInput() + " is."));
							result.add(new EmotionAction("sad"));
							result.add(new SpeechAction(
									"Let me try again. Please tell me the object yout want me to guess."));
							state = GuessingGameState.REGISTER_STOP_WORDS;
							return result;
						}
						result.add(
								new SpeechAction("I see " + term + ". But I will forget that you told me what it is."));
						result.add(new EmotionAction("happy"));// TODO make him
																// twinkle
					} else {
						result.add(new SpeechAction("I can't see " + sentence.getInput()));
						result.add(new EmotionAction("sad"));
						result.add(
								new SpeechAction("Let me see again. Please tell me the object yout want me to guess."));
						state = GuessingGameState.REGISTER_STOP_WORDS;
						return result;
					}
				}
			} else {
				if (term.equals("")) {
					result.add(new SpeechAction("I can't see anything"));
					result.add(new EmotionAction("sad"));
					result.add(new SpeechAction("Let me see again. Please tell me the object yout want me to guess."));
					state = GuessingGameState.REGISTER_STOP_WORDS;
					return result;
				}
			}

			// Ask for stopwords
			result.add(new SpeechAction("Please tell me words that should not be used to describe the object."));
			state = GuessingGameState.FINALIZE_SETUP;
			return result;

		case FINALIZE_SETUP:
			// save the stopwords
			if (sentence != null) {
				String[] arr = sentence.getInput().split("\\s+");
				// the term is also not allowed to be said
				stopWords.add(term);
				String allStopWords = term;
				for (int i = 0; i < arr.length; ++i) {
					arr[i] = arr[i].replaceAll("\\s+|,", "").toLowerCase();
					stopWords.add(arr[i]);
					allStopWords = allStopWords + ", " + arr[i];
				}
				result.add(new SpeechAction("The words you are not allowed to use are: " + allStopWords + "."));
			}
			if (groupAsTurnFirst) {
				result.add(new SpeechAction(groupA + ", please describe the object."));
				state = GuessingGameState.GROUP_A_SENTENCE_INPUT;
				return result;
			} else {
				result.add(new SpeechAction(groupB + ", please describe the object."));
				state = GuessingGameState.GROUP_B_SENTENCE_INPUT;
				return result;
			}
		case GROUP_A_SENTENCE_INPUT:
			if (sentence == null) {
				result.add(new SpeechAction(groupA + " didn't even try."));
				result.add(new EmotionAction("sad"));
				if (!groupAsTurnFirst) {
					// groupB started this round -> nobody could answer right
					return nobodyGotIt(result);
				} else {
					// groupA started this round -> groupBs turn now
					result.add(new SpeechAction(groupB + ", please describe the object."));
					state = GuessingGameState.GROUP_B_SENTENCE_INPUT;
					return result;
				}
			} else {
				List<String> hintsList = Arrays.asList(sentence.getInput().split("\\s+"));
				for (int i = 0; i < hintsList.size(); ++i) {
					hintsList.set(i, hintsList.get(i).replaceAll("\\s+|,", ""));
					if (stopWords.contains(hintsList.get(i).toLowerCase())) {
						// They used a stopWord
						result.add(new SpeechAction(groupA + " used " + hintsList.get(i).toLowerCase()
								+ " which is a word that was not allowed. The other team will get a point."));
						result.add(new EmotionAction("sad"));
						addPoint(GROUP_B);
						result.add(new SpeechAction("Do you want to play antoher round?"));
						state = GuessingGameState.REGISTER_TERM;
						return result;
					}
				}
				probTeamA = association.getProbability(hintsList);

				// TODO set envVariables message='line1|line2' and
				// (optional)color='42 23 12'
				// calculate an output for edison (0-1 how close they got);
				// call python script to send it to edison

				double outputEdison = probTeamA / POINT_BORDER;
				if (outputEdison > 1) {
					outputEdison = 1;
				}

				String command = ".." + File.separator + "display_client.py"; // TODO
																				// insert
				// pythonscript path
				// here
				String message = "message=" + groupA + ": " + String.format("%4.3f", outputEdison) + "|" + groupB
						+ String.format("%4.3f", (probTeamB / POINT_BORDER)) + "     |";
				String[] envArgs = new String[2];
				envArgs[0] = message;
				envArgs[1] = "color=99 17 27";
				// TODO color

				if (!command.equals("")) {
					try {
						Runtime.getRuntime().exec(command, envArgs);
						System.out.println("Ran python");
					} catch (IOException e) {
						e.printStackTrace();
					}
				}

				if (probTeamA < POINT_BORDER) {
					// score too low
					if (!groupAsTurnFirst) {
						// groupB started this round -> nobody answered right
						return nobodyGotIt(result);
					} else {
						// groupA started this round -> groupBs turn now
						result.add(new SpeechAction(
								groupA + ", your hints didn't really make clear to me what you were talking about."));
						result.add(new SpeechAction(groupB + ", please describe the object."));
						state = GuessingGameState.GROUP_B_SENTENCE_INPUT;
						return result;
					}
				} else {
					// score high enough -> point for groupA
					result.add(
							new SpeechAction(groupA + ", you clearly described " + term + ". You will get one point."));
					result.add(new EmotionAction("happy"));
					addPoint(GROUP_A);
					result.add(new SpeechAction("Do you want to play antoher round?"));
					state = GuessingGameState.REGISTER_TERM;
					return result;
				}
			}
		case GROUP_B_SENTENCE_INPUT:
			if (sentence == null) {
				result.add(new SpeechAction(groupB + " didn't even try."));
				result.add(new EmotionAction("sad"));
				if (!groupAsTurnFirst) {
					// groupB started this round -> groupAs turn now
					result.add(new SpeechAction(groupA + ", please describe the object."));
					state = GuessingGameState.GROUP_A_SENTENCE_INPUT;
					return result;
				} else {
					// groupA started this round -> nobody got it
					return nobodyGotIt(result);
				}
			} else {
				List<String> hintsList = Arrays.asList(sentence.getInput().split("\\s+"));
				for (int i = 0; i < hintsList.size(); ++i) {
					hintsList.set(i, hintsList.get(i).replaceAll("\\s+|,", ""));
					if (stopWords.contains(hintsList.get(i).toLowerCase())) {
						// They used a stopWord
						result.add(new SpeechAction(groupB + " used " + hintsList.get(i).toLowerCase()
								+ " which is a word that was not allowed. The other team will get a point."));
						result.add(new EmotionAction("sad"));
						addPoint(GROUP_A);
						result.add(new SpeechAction("Do you want to play antoher round?"));
						state = GuessingGameState.REGISTER_TERM;
						return result;
					}
				}
				probTeamB = association.getProbability(hintsList);
				if (probTeamB < POINT_BORDER) {
					// score too low
					if (!groupAsTurnFirst) {
						// groupB started this round -> groupAs turn now
						result.add(new SpeechAction(
								groupB + ", your hints didn't really make clear to me what you were talking about."));
						result.add(new SpeechAction(groupA + ", please describe the object."));
						state = GuessingGameState.GROUP_A_SENTENCE_INPUT;
						return result;
					} else {
						// groupA started this round -> nobody answered right
						return nobodyGotIt(result);
					}
				} else {
					// score high enough -> point for groupB
					result.add(
							new SpeechAction(groupB + ", you clearly described " + term + ". You will get one point."));
					result.add(new EmotionAction("happy"));
					addPoint(GROUP_B);
					result.add(new SpeechAction("Do you want to play antoher round?"));
					state = GuessingGameState.REGISTER_TERM;
					return result;
				}
			}
		case TELL_RESULTS:
			List<Action> lastwords = new ArrayList<Action>();
			if (pointsA > pointsB) {
				// group A won
				lastwords.add(new SpeechAction("The winner is " + groupA + " with " + pointsA + " points. " + groupB
						+ " got " + pointsB + " points."));
			} else if (pointsA < pointsB) {
				// group B won
				lastwords.add(new SpeechAction("The winner is " + groupB + " with " + pointsB + " points. " + groupA
						+ " got " + pointsA + " points."));
			} else {
				// tied
				lastwords.add(new SpeechAction(groupA + " and " + groupB + " both got " + pointsA + " points."));
			}
			result.add(new ShutDownAction(lastwords));
			return result;

		default:
			List<Action> mylastwords = new ArrayList<Action>();
			mylastwords.add(new SpeechAction("I am lost, bye."));
			result.add(new ShutDownAction(mylastwords));
			return result;
		}
	}

	public void setSentence(SentenceInput sentence) {
		this.sentence = sentence;
	}

	public void setClassification(ClassificationInput classification) {
		this.classification = classification;
	}

	private Association getAssociations(String object) {
		String[] arrNouns, arrVerbs, arrAdj;
		// read Files in /ratings
		String filePath = ".." + File.separator + "ratings" + File.separator + object.toLowerCase() + ".txt"; // TODO
																												// change
																												// path
		try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
			arrNouns = br.readLine().split("\\s+");
			arrVerbs = br.readLine().split("\\s+");
			arrAdj = br.readLine().split("\\s+");
			br.close();
		} catch (FileNotFoundException e2) {
			e2.printStackTrace();
			return null;
		} catch (IOException e1) {
			e1.printStackTrace();
			return null;
		}
		Association objAssociations = new Association(Arrays.asList(arrNouns), Arrays.asList(arrVerbs),
				Arrays.asList(arrAdj));
		return objAssociations;
	}

	private void addPoint(int group) {
		if (group == GROUP_A) {
			++pointsA;
		} else {
			++pointsB;
		}
	}

	private List<Action> nobodyGotIt(List<Action> result) {
		result.add(new SpeechAction("Nobody described the Object good enough for me to recognize it. "
				+ "Maybe I am just not smart enough"));
		result.add(new EmotionAction("sad"));
		result.add(new SpeechAction("Do you want to play antoher round?"));
		state = GuessingGameState.REGISTER_TERM;
		return result;
	}

	private void filterSpamWords() {
		// TODO take out words like 'and', 'or', .. of the sentence and return
		// the filtered sentence
		// filter with POS: UH, ..
	}

}
