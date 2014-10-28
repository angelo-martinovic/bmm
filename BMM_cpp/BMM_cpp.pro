QMAKE_CXXFLAGS_DEBUG += -O0 -ggdb -march=corei7 -mfpmath=sse -mtune=corei7 -std=c++0x -DSINGLETHREADED

QMAKE_CXXFLAGS_RELEASE += -O3 -march=corei7 -mfpmath=sse -mtune=corei7 -std=c++0x -fopenmp

QT     -= gui core
LIBS   -= -lQtGui -lQtCore

HEADERS += \
    Grammar/Symbol.h \
    Grammar/Production.h \
    Grammar/Grammar2D.h \
    Grammar/Attribute.h \
    LatticeParser/EarleyState2D.h \
    LatticeParser/EarleyState2DSet.h \
    LatticeParser/EarleyParser2D.h \
    LatticeParser/ParsingTools.h \
    tree-2.81/tree_util.hh \
    tree-2.81/tree.hh \
    tree-2.81/xinlin.hh \
    ImageParser/Scope2D.h \
    ImageParser/ImageTools.h \
    ImageParser/Node.h \
    ImageParser/TreeDerivation.h \
    ImageParser/EnergyFunctions.h \
    ImageParser/OptimizationFunctions.h \
    ImageParser/Evaluator.h \
    UnitTests/UnitTests.h \
    ModelMerging/ModelMerging.h \
    Experiments/Experiments.h \
    IO/IO.h \
    Corpus.h \
    ProgramOptions.h \
    LatticeParser/Visualization.h \
    Printable.h

SOURCES += \
    Grammar/Symbol.cpp \
    Grammar/Attribute.cpp \
    Grammar/Production.cpp \
    Grammar/Grammar2D.cpp \
    LatticeParser/EarleyState2D.cpp \
    LatticeParser/EarleyState2DSet.cpp \
    LatticeParser/EarleyParser2D.cpp \
    LatticeParser/ParsingTools.cpp \
    ImageParser/Scope2D.cpp \
    ImageParser/ImageTools.cpp \
    ImageParser/Node.cpp \
    ImageParser/TreeDerivation.cpp \
    ImageParser/EnergyFunctions.cpp \
    ImageParser/RandomWalk.cpp \ 
    ImageParser/Evaluator.cpp \
    IO/PrepareTrainAndTestData.cpp \
    IO/LoadImage.cpp \
    ModelMerging/ModelMerging.cpp \
    ModelMerging/DataIncorporation.cpp \
    UnitTests/Hayko.cpp \
    UnitTests/UnitTest1.cpp \
    UnitTests/UnitTest2.cpp \
    UnitTests/UnitTest3.cpp \
    UnitTests/UnitTest4.cpp \
    UnitTests/UnitTest5.cpp \
    Corpus.cpp \
    ProgramOptions.cpp \
    Experiments/RunGrammarInduction.cpp \
    Experiments/RunGrammarSampling.cpp \
    Experiments/Experiments.cpp \
    main.cpp \
    IO/IO.cpp \
    Experiments/RunImageParsing.cpp \
    Grammar/SampleDerivation.cpp \
    LatticeParser/Visualization.cpp \
    ModelMerging/MergeNonTerminals.cpp \
    ModelMerging/ChunkNonTerminals.cpp \
    ModelMerging/ModelConversion.cpp

QMAKE_LIBS = -Wl,-rpath,./opencv2.4/lib \
            -lopencv_core \
            -lopencv_imgproc \
            -lopencv_highgui \
            -lboost_system \
            -lboost_filesystem \
            -lboost_program_options \
            -lboost_serialization \
            -fopenmp

QMAKE_LIBDIR = ./opencv2.4/lib



INCLUDEPATH = ./opencv2.4/include

OTHER_FILES +=




