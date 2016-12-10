package de.roboy.dialog.action;

public class EmotionAction implements Action {
	private String emotion;
	
	public EmotionAction(String emotion){
		this.emotion = emotion;
	}
	public String getEmotion(){
		return emotion;
	}
}
