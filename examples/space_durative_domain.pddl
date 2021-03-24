(define (domain space)
(:requirements :strips :equality :typing :durative-actions :negative-preconditions :universal-preconditions :adl)

(:types
  robot
  spaceship
  planet

)
(:predicates
  (robotAt ?r - robot ?p - planet)
  (robotIn ?r -robot ?s - spaceship)
  (spaceshipAt ?s - spaceship ?p - planet)
  (allOK ?r - robot)
  (spaceshipFree ?s - spaceship)
)

(:durative-action Load
  :parameters (?r - robot ?s - spaceship ?p - planet)
  :duration (= ?duration 2)
  :condition (and
      (at start (and
      	(spaceshipFree ?s)
        (robotAt ?r ?p)

      ))
      (over all (and
        (spaceshipAt ?s ?p)
      ))
    )
  :effect (and
      (at start (and
      	(not (spaceshipFree ?s))
        (not (robotAt ?r ?p))
      ))
      (at end (and
        (robotIn ?r ?s)
        (spaceshipFree ?s)
      ))
    )
)

(:durative-action Unload
  :parameters (?r - robot ?s - spaceship ?p - planet)
  :duration (= ?duration 2)
  :condition (and
    (at start (and
      (robotIn ?r ?s)
      (spaceshipAt ?s ?p)
      (spaceshipFree ?s)
    ))
    )
  :effect(and
    (at start (and
    	(not (spaceshipFree ?s))
        (not (robotIn ?r ?s))
      ))
    (at end (and
        (robotAt ?r ?p)
        (spaceshipFree ?s)
      ))
    )
)

(:durative-action Fly
  :parameters (?s - spaceship ?from - planet ?to - planet)
  :duration (= ?duration 5)
  :condition(and
    (at start (and
	(spaceshipAt ?s ?from)
	(not (spaceshipAt ?s ?to))
	(spaceshipFree ?s)
    ))
    )
  :effect(and
    (at start (and
    	(not (spaceshipFree ?s))
        (not (spaceshipAt ?s ?from))
      ))
      (at end (and
        (spaceshipAt ?s ?to)
        (spaceshipFree ?s)
      ))
    )
)

(:durative-action test
  :parameters (?s - spaceship ?r - robot ?p - planet)
  :duration (= ?duration 1)
  :condition(and
     (at start (and
	  (spaceshipAt ?s ?p)
	  (spaceshipFree ?s)
          (or
	     (robotIn ?r ?s)
	     (robotAt ?r ?p)
         ))
    )
    )
  :effect(and
    (at start (and
    	(not (spaceshipFree ?s))
    ))
    (at end (and
    	(spaceshipFree ?s)
        (allOK ?r)
      ))
    )
)
)
