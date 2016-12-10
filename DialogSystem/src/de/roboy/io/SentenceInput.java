package de.roboy.io;

public class SentenceInput implements Input {
	private String input;
	public SentenceInput(String input){
		this.input = input;
	}
	public String getInput(){
		return input;
	}
}
