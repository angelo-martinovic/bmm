==============================================================
BMM : Bayesian Model Merging for Inverse Procedural Modeling
==============================================================

Source code of the paper

* Hierarchical Co-Segmentation of Building Facades
   `Anđelo Martinović <http://homes.esat.kuleuven.be/~amartino/>`_, Luc Van Gool; to appear at 3DV 2014


**Abstract**
In this work, we introduce a new system for automatic discovery of high-level structural representations of building facades. Under the assumption that each facade can be represented as a hierarchy of rectilinear subdivisions, our goal is to find the optimal direction of splitting, along with the number and positions of the split lines at each level of the tree. Unlike previous approaches, where each facade is analysed in isolation, we propose a joint analysis of a set of facade images. Initially, a co-segmentation approach is used to produce consistent decompositions across all facade images. Afterwards, a clustering step identifies semantically similar segments. Each cluster of similar segments is then used as the input for the joint segmentation in the next level of the hierarchy. We show that our approach produces consistent hierarchical segmentations on two different facade datasets. Furthermore, we argue that the discovered hierarchies capture essential structural information, which is demonstrated on the tasks of facade retrieval and virtual facade synthesis.