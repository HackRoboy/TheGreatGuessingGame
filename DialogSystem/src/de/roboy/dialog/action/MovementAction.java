package de.roboy.dialog.action;

public class MovementAction implements Action {
	private String movement;

	public MovementAction(String movement){
		this.movement = movement;
	}

	public String getMovement() {
		return movement;
	}
}
