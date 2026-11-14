^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap4r2_qualisys_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* Package renamed qualisys_driver -> mocap4r2_qualisys_driver and rewritten on MocapSystemNode (controlled by the CLI and the TUI).
* Qualisys C++ SDK (RT protocol 1.28, MIT) included instead of fetched at build time.
* All frames streamed (TCP or UDP) to a dedicated thread; untracked bodies flagged instead of NaN; any QTM up axis converted to Z-up; skeletons in global coordinates.
* Fixed timer reset on cleanup, forced use_sim_time and wall clock stamps.
* Contributors: Francisco Martín Rico, Emir Cem Gezer, José Miguel Guerrero, Juan Carlos Manzanares Serrano, Lorena Bajo Rebollo, Chao Yao, SkogsTomte, suet
