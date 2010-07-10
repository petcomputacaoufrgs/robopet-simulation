TARGETS= ../Library/libbox2d.a

ifdef DEVKITPRO
TARGETS += Gen/nds-float/lib/libbox2d.a
endif

CXXFLAGS=	-g -O2

SOURCES = \
	./Dynamics/b2Body.cpp \
	./Dynamics/b2Island.cpp \
	./Dynamics/b2World.cpp \
	./Dynamics/b2ContactManager.cpp \
	./Dynamics/Contacts/b2Contact.cpp \
	./Dynamics/Contacts/b2PolyContact.cpp \
	./Dynamics/Contacts/b2CircleContact.cpp \
	./Dynamics/Contacts/b2PolyAndCircleContact.cpp \
	./Dynamics/Contacts/b2ContactSolver.cpp \
	./Dynamics/b2WorldCallbacks.cpp \
	./Dynamics/Joints/b2MouseJoint.cpp \
	./Dynamics/Joints/b2PulleyJoint.cpp \
	./Dynamics/Joints/b2Joint.cpp \
	./Dynamics/Joints/b2RevoluteJoint.cpp \
	./Dynamics/Joints/b2PrismaticJoint.cpp \
	./Dynamics/Joints/b2DistanceJoint.cpp \
	./Dynamics/Joints/b2GearJoint.cpp \
	./Common/b2StackAllocator.cpp \
	./Common/b2Math.cpp \
	./Common/b2BlockAllocator.cpp \
	./Common/b2Settings.cpp \
	./Collision/b2Collision.cpp \
	./Collision/b2Distance.cpp \
	./Collision/Shapes/b2Shape.cpp \
	./Collision/Shapes/b2CircleShape.cpp \
	./Collision/Shapes/b2PolygonShape.cpp \
	./Collision/b2TimeOfImpact.cpp \
	./Collision/b2PairManager.cpp \
	./Collision/b2CollidePoly.cpp \
	./Collision/b2CollideCircle.cpp \
	./Collision/b2BroadPhase.cpp 
#	./Contrib/b2Polygon.cpp \
#	./Contrib/b2Triangle.cpp


ifneq ($(INCLUDE_DEPENDENCIES),yes)

all:	
	@make --no-print-directory INCLUDE_DEPENDENCIES=yes $(TARGETS)

.PHONY:	clean
clean:
	rm -rf Gen

else

-include $(addprefix Gen/,$(SOURCES:.cpp=.d))

ifdef DEVKITPRO
-include $(addprefix Gen/nds-float/,$(SOURCES:.cpp=.d))
endif

endif

Gen/%.o:		%.cpp
	@mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -c -o $@ $<


../Library/libbox2d.a:	$(addprefix Gen/,$(SOURCES:.cpp=.o))
	ar cr $@ $^
	ranlib $@ 

Gen/%.d:		%.cpp
	@mkdir -p $(dir $@)
	c++ -MM -MT $(@:.d=.o) $(CXXFLAGS) -o $@ $<


NDS_COMPILE_FLAGS= -g -O2 -fomit-frame-pointer -ffast-math \
		-march=armv5te -mtune=arm946e-s -mthumb-interwork \
		-DARM9 -fno-rtti -fno-exceptions -c \
		-DTARGET_IS_NDS \
		-I$(DEVKITPRO)/libnds/include 

Gen/nds-float/%.o:		%.cpp
	@mkdir -p $(dir $@)
	arm-eabi-g++ $(NDS_COMPILE_FLAGS) -o $@ $<

Gen/nds-float/lib/libbox2d.a:	$(addprefix Gen/nds-float/,$(SOURCES:.cpp=.o))
	@mkdir -p $(dir $@)
	arm-eabi-ar cr $@ $^
	arm-eabi-ranlib $@ 

Gen/nds-float/%.d:		%.cpp
	@mkdir -p $(dir $@) 
	arm-eabi-g++ -MM -MT $(@:.d=.o) $(NDS_COMPILE_FLAGS) -o $@ $<



