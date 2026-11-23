#include <std_include.hpp>
#include "loader/component_loader.hpp"

#include "gsc/script_extension.hpp"

#include "game/game.hpp"
#include "game/dvars.hpp"

#include <utils/csv.hpp>
#include <utils/hook.hpp>

#include "console.hpp"
#include "filesystem.hpp"

namespace pathnodes
{
	namespace
	{
		game::dvar_t* scr_enable_jump_nodes = nullptr;
		game::dvar_t* scr_load_csv_paths = nullptr;

		utils::memory::allocator path_allocator;

		static const std::unordered_map<std::string, unsigned short> waypoint_types =
		{
			{"stand", 13},
			{"crouch", 14},
			{"prone", 15},
		};

		scripting::script_value mark_dangerous_nodes(const gsc::function_args& args)
		{
			if (args.size() < 3)
			{
				throw std::runtime_error("invalid args");
			}

			const auto origin = args[0].as<scripting::vector>();
			const auto radius = args[1].as<float>();

			if (radius <= 0.f)
			{
				throw std::runtime_error("radius must be > 0");
			}

			const auto dangerous = args[2].as<int>();
			game::SV_BotMarkNodesAsDangerous(origin, radius, dangerous);
			return {};
		}

		scripting::script_value mark_dangerous_nodes_in_trigger(const gsc::function_args& args)
		{
			if (args.size() < 2)
			{
				throw std::runtime_error("invalid args");
			}

			const auto entity = args[0].as<scripting::entity>();
			const auto dangerous = args[1].as<int>();

			const auto entref = entity.get_entity_reference();
			if (entref.classnum != 0)
			{
				throw std::runtime_error("invalid entity");
			}

			const auto trigger = &game::mp::g_entities[entref.entnum];

			const auto x1 = std::abs(trigger->r.box.midPoint[0]);
			const auto y1 = std::abs(trigger->r.box.midPoint[1]);

			const auto max_height = ((trigger->r.box.halfSize[2] + trigger->r.box.halfSize[2]) + 256.f) + 1.f;

			const auto distance = std::sqrtf(
				(y1 + trigger->r.box.halfSize[1]) * (y1 + trigger->r.box.halfSize[1]) +
				(x1 + trigger->r.box.halfSize[0]) * (x1 + trigger->r.box.halfSize[0])
			);

			game::pathsort_s nodes[256]{};
			const auto count = game::Path_NodesInCylinder(trigger->r.currentOrigin, nullptr, (distance + 256.f) + 1.f, max_height, nodes, 256, -1);

			float midpoint[3]{};
			float halfsize[3]{};

			midpoint[0] = trigger->r.absBox.midPoint[0];
			midpoint[1] = trigger->r.absBox.midPoint[1];
			midpoint[2] = trigger->r.absBox.midPoint[2];

			halfsize[0] = trigger->r.absBox.halfSize[0] + 128.f;
			halfsize[1] = trigger->r.absBox.halfSize[1] + 128.f;
			halfsize[2] = trigger->r.absBox.halfSize[2] + 128.f;

			for (auto i = 0; i < count; i++)
			{
				const auto node = nodes[i].node;
				float pos[3]{};
				pos[0] = node->constant.vLocalOrigin[0];
				pos[1] = node->constant.vLocalOrigin[1];
				pos[2] = node->constant.vLocalOrigin[2];
			
				game::WorldifyPosFromParent(node, pos);

				const auto delta_x = std::abs(midpoint[0] - pos[0]);
				const auto delta_y = std::abs(midpoint[1] - pos[1]);
				const auto delta_z = std::abs(midpoint[2] - pos[2]);
				const auto is_in = (delta_x <= halfsize[0]) && (delta_y <= halfsize[1]) && (delta_z <= halfsize[2]);

				if (!is_in)
				{
					continue;
				}

				if (dangerous)
				{
					++node->dynamic.dangerousCount;
					node->dynamic.flags |= 1u;
				}
				else
				{
					if (node->dynamic.dangerousCount > 0)
					{
						node->dynamic.dangerousCount--;
					}

					if (node->dynamic.dangerousCount == 0)
					{
						node->dynamic.flags &= 0xFFFEu;
					}
				}
			}

			return {};
		}
		struct pathpoint_t
		{
			float vOrigPoint[3];
			float fDir2D[2];
			float fOrigLength;
			int iNodeNum;
		};

		struct path_s
		{
			pathpoint_t pts[32];
			char wPathLen;
			char wOrigPathLen;
			char wDodgeCount;
			char wNegotiationStartNode;
			char lookaheadNextNode;
			char pathChangeNotifyNode;
			__int16 wDodgeEntity;
			float vFinalGoal[3];
			float vStartPos[3];
			float lookaheadDir[3];
			float lookaheadPos[3];
			float fLookaheadDist;
			float fLookaheadAmount;
			float fLookaheadDistToNextNode;
			int minLookAheadNodes;
			int flags;
			int iPathTime;
			int iPathClearedTime;
			int eTeam;
			float fCurrLength;
			float vCurrPoint[3];
			int iPathEndTime;
			float pathEndAnimDistSq;
			bool pathEndAnimNotified;
			bool lookaheadHitsStairs;
			bool useChokePoints;
			float pathChangeTracePos[2];
			int randomPercent;
			unsigned int parentIndex;
			float badPlaceAwareness;
			__int16 ownerEntNum;
			int iTraceMask;
		};

		struct PathFindInput
		{
			path_s* pPath;
			float* vStartPos;
			float* vGoalPos;
			game::pathnode_t* pNodeFrom;
			game::pathnode_t* pNodeTo;
			int bAllowNegotiationLinks;
			int bAllowLadderNodes;
			int useChokePoints;
			__int16 searchEntNum;
			float badplacePercent;
			int avoidExpStances;
			game::pathnode_t* avoidExpPosNode[5];
			float avoidExpPosDir[5][3];
			float avoidExpPosDot[5];
			float avoidExpPosPercent[5];
			float avoidExpPosRadiusSq[5];
			float avoidExpSkyPercent;
			float avoidExpGenPercent;
			float avoidExpLOSPercent;
			float avoidExpTrafficPercent;
			unsigned __int16* pathZoneCosts;
			int pathZoneCostsCount;
			float pathSpreadPercent;
			float avoidLocationPos[3];
			float avoidLocationPercent;
			float avoidLocationMaxDist;
			float traversalCostPercentClassic;
			float traversalCostPercentBoost;
		};

		struct CustomSearchInfo_FindPathAlien
		{
			game::pathnode_t* m_pNodeTo;
			float startPos[3];
			int randomWeightIndex;
			float randomNodeCosts[4];
			float chokePointWeights[4];
			float negotiationOverlapCost;
			float jumpCost;
			float runCost;
			float traverseCost;
			float dangerousCost;
			float wallrunCost;
		};

		//enum ScrAgent_PathTraversalPermissions : std::int32_t
		//{
		//	SA_PATH_ALLOW_CROUCH = 0x1,
		//	SA_PATH_ALLOW_PRONE = 0x2,
		//	SA_PATH_ALLOW_JUMP = 0x4,
		//	SA_PATH_ALLOW_LADDERS = 0x8,
		//};

		enum ScrAgent_PathTraversalPermissions : std::int32_t
		{
			SA_PATH_ALLOW_JUMP = 0x1,
			SA_PATH_ALLOW_LADDERS = 0x2,
			SA_PATH_ALLOW_CROUCH = 0x4,
			SA_PATH_ALLOW_PRONE = 0x8,
		};

		struct scripted_agent_info_t
		{
			char __pad0[5256];
			float jumpCost;
			float traverseCost;
			float runCost;
			char __pad1[76];
			unsigned __int16 pathTraversalPermissions;
		}; // size = 16352

		assert_offsetof(scripted_agent_info_t, jumpCost, 5256);

		struct scripted_agent_custom_info_t
		{
			float wallRunCost;
		};

		bool bAllowJumpLinks = false;

		scripted_agent_custom_info_t scr_agent_custom_info[24]{};

		scripted_agent_custom_info_t* get_scripted_agent_custom_info(game::mp::gentity_s* ent)
		{
			return &scr_agent_custom_info[ent->s.number - 0x12];
		}

		int Path_CheckFromAndToNodes(PathFindInput* pathFindInput)
		{
			return utils::hook::invoke<int>(0x3EA7C0_b, pathFindInput);
		}

		int G_rand()
		{
			return utils::hook::invoke<int>(0x45FA50_b);
		}

		scripted_agent_info_t* Agent_GetScriptedAgentInfo(const game::mp::gentity_s* a1)
		{
			return utils::hook::invoke<scripted_agent_info_t*>(0x517B30_b, a1);
		}

		int Path_GetPathRandomness(PathFindInput* pathFindInput)
		{
			return utils::hook::invoke<int>(0x3EB3E0_b, pathFindInput);
		}

		void CustomSearchInfo_FindPathAlien_CustomSearchInfo_FindPathAlien(CustomSearchInfo_FindPathAlien* this_, PathFindInput* pathFindInput)
		{
			const auto agent_info = Agent_GetScriptedAgentInfo(&game::mp::g_entities[pathFindInput->searchEntNum]);
			const auto agent_custom_info = get_scripted_agent_custom_info(&game::mp::g_entities[pathFindInput->searchEntNum]);

			this_->jumpCost = agent_info->jumpCost;
			this_->runCost = agent_info->runCost;
			this_->dangerousCost = 750.0;
			this_->traverseCost = agent_info->traverseCost;
			this_->wallrunCost = agent_custom_info->wallRunCost;

			const auto randomness = Path_GetPathRandomness(pathFindInput);

			this_->randomWeightIndex = 0;
			if (randomness)
			{
				for (auto i = 0; i < 4; i++)
				{
					this_->randomNodeCosts[i] = (G_rand() % randomness) * 5.1199999f;
				}
			}
			else
			{
				this_->randomNodeCosts[0] = 0.f;
				this_->randomNodeCosts[1] = 0.f;
				this_->randomNodeCosts[2] = 0.f;
				this_->randomNodeCosts[3] = 0.f;
			}

			if (pathFindInput->useChokePoints)
			{
				const auto rand = G_rand();
				const auto flags = ~(1 << (G_rand() % 4)) & rand;

				if ((flags & 1) != 0)
				{
					this_->chokePointWeights[0] = 1024.f;
				}
				else
				{
					this_->chokePointWeights[0] = 0.f;
				}

				if ((flags & 2) != 0)
				{
					this_->chokePointWeights[1] = 1024.f;
				}
				else
				{
					this_->chokePointWeights[1] = 0.f;
				}

				if ((flags & 4) != 0)
				{
					this_->chokePointWeights[2] = 1024.f;
				}
				else
				{
					this_->chokePointWeights[2] = 0.f;
				}

				if ((flags & 8) == 0)
				{
					this_->chokePointWeights[3] = 0.f;
				}
			}
			else
			{
				this_->chokePointWeights[0] = 0.f;
				this_->chokePointWeights[1] = 0.f;
				this_->chokePointWeights[2] = 0.f;
				this_->chokePointWeights[3] = 0.f;
			}

			this_->negotiationOverlapCost = 1500.0;
		}

		game::pathnode_t* Path_ConvertIndexToNode(int index)
		{
			return utils::hook::invoke<game::pathnode_t*>(0x3F7600_b, index);
		}

		enum nodeType : std::uint16_t
		{
			NODE_ERROR = 0,
			NODE_PATHNODE = 1,
			NODE_COVER_STAND = 2,
			NODE_COVER_CROUCH = 3,
			NODE_COVER_CROUCH_WINDOW = 4,
			NODE_COVER_PRONE = 5,
			NODE_COVER_RIGHT = 6,
			NODE_COVER_LEFT = 7,
			NODE_COVER_WIDE_RIGHT = 8,
			NODE_COVER_WIDE_LEFT = 9,
			NODE_COVER_MULTI = 10,
			NODE_AMBUSH = 11,
			NODE_EXPOSED = 12,
			NODE_CONCEALMENT_STAND = 13,
			NODE_CONCEALMENT_CROUCH = 14,
			NODE_CONCEALMENT_PRONE = 15,
			NODE_DOOR = 16,
			NODE_DOOR_INTERIOR = 17,
			NODE_SCRIPTED = 18,
			NODE_NEGOTIATION_BEGIN = 19,
			NODE_NEGOTIATION_END = 20,
			NODE_TURRET = 21,
			NODE_GUARD = 22,
			NODE_PATHNODE_3D = 23,
			NODE_COVER_UP_3D = 24,
			NODE_COVER_RIGHT_3D = 25,
			NODE_COVER_LEFT_3D = 26,
			NODE_EXPOSED_3D = 27,
			NODE_SCRIPTED_3D = 28,
			NODE_NEGOTIATION_BEGIN_3D = 29,
			NODE_NEGOTIATION_END_3D = 30,
			NODE_NUMTYPES = 31,
			NODE_DONTLINK = 31,
		};

		constexpr const auto node_type_jump = 32;
		constexpr const auto node_type_jump_attack = 33;
		constexpr const auto node_type_end = 34;

		bool is_jump_node(const std::uint16_t type)
		{
			return type == node_type_jump || type == node_type_jump_attack;
		}

		bool is_traverse_begin_node(const std::uint16_t type)
		{
			return type == game::NODE_NEGOTIATION_BEGIN || type == game::NODE_NEGOTIATION_BEGIN_3D;
		}

		bool is_traverse_end_node(const std::uint16_t type)
		{
			return type == game::NODE_NEGOTIATION_END || type == game::NODE_NEGOTIATION_END_3D;
		}

		bool is_traverse_begin_or_jump_node(const std::uint16_t type)
		{
			return is_traverse_begin_node(type) || is_jump_node(type);
		}

		bool is_traverse_end_or_jump_node(const std::uint16_t type)
		{
			return is_traverse_end_node(type) || is_jump_node(type);
		}

		float CustomSearchInfo_FindPathAlien_GetLinkCost(CustomSearchInfo_FindPathAlien* this_, game::pathnode_t* pCurrent, int link_index)
		{
			const auto linked = Path_ConvertIndexToNode(pCurrent->constant.Links[link_index].nodeNum);

			auto cost = 0.f;

			if (is_jump_node(pCurrent->constant.type) && pCurrent->constant.target != 0
				&& is_jump_node(linked->constant.type) && linked->constant.target && pCurrent->constant.target == linked->constant.targetname)
			{
				cost = this_->wallrunCost;
			}
			else if (pCurrent->constant.type == NODE_DONTLINK || linked->constant.type == NODE_DONTLINK)
			{
				if (is_traverse_begin_or_jump_node(pCurrent->constant.type) && is_traverse_end_or_jump_node(linked->constant.type))
				{
					cost = this_->traverseCost;
				}
				else
				{
					cost = this_->runCost;
				}
			}
			else
			{
				cost = this_->jumpCost;
			}

			return cost * pCurrent->constant.Links[link_index].fDist;
		}

		float CustomSearchInfo_FindPathAlien_GetNodeCost(CustomSearchInfo_FindPathAlien* this_, game::pathnode_t* pSuccessor, const float* vGoalPos)
		{
			const auto v5 = sqrtf(
				(((this_->startPos[1] - pSuccessor->constant.vLocalOrigin[1]) * (this_->startPos[1] - pSuccessor->constant.vLocalOrigin[1])) +
					((this_->startPos[0] - pSuccessor->constant.vLocalOrigin[0]) * (this_->startPos[0] - pSuccessor->constant.vLocalOrigin[0]))) +
				((this_->startPos[2] - pSuccessor->constant.vLocalOrigin[2]) * (this_->startPos[2] - pSuccessor->constant.vLocalOrigin[2])));

			const auto min_use_dist = pSuccessor->constant.___u11.minUseDistSq;
			auto result = static_cast<float>(pSuccessor->dynamic.userCount) * this_->negotiationOverlapCost;

			if (min_use_dist > 1.0 && min_use_dist > (v5 * v5))
			{
				result += this_->negotiationOverlapCost;
			}

			if ((pSuccessor->constant.spawnflags & 0x20) != 0)
			{
				result += this_->chokePointWeights[this_->randomWeightIndex];
			}

			if ((pSuccessor->dynamic.flags & 1) != 0)
			{
				result += (pSuccessor->dynamic.dangerousCount * this_->dangerousCost);
			}

			if (pSuccessor->dynamic.pOwner.number)
			{
				result += 1500.0f;
			}

			if ((pSuccessor->constant.spawnflags & 4) != 0 && pSuccessor->constant.type == 1)
			{
				if (result < 256.0f)
				{
					result = 256.f;
				}

				result *= 2.0f;
			}

			this_->randomWeightIndex = (this_->randomWeightIndex + 1) % 4;

			return result;
		}

		struct PathHeap
		{
			unsigned __int16 index[8192];
			unsigned __int16 nodes[8192];
			unsigned int size;
		};

		enum team_t
		{
			TEAM_FREE = 0x0,
			TEAM_BAD = 0x0,
			TEAM_AXIS = 0x1,
			TEAM_ALLIES = 0x2,
			TEAM_SPECTATOR = 0x3,
			TEAM_NUM_TEAMS = 0x4,
		};

		void pathnode_t_LocalizePosToParent(game::pathnode_t* this_, float* const a2)
		{
			utils::hook::invoke<void>(0x3F6890_b, this_, a2);
		}

		int Path_ConvertNodeToIndex(const game::pathnode_t* a1)
		{
			return utils::hook::invoke<int>(0x3F7620_b, a1);
		}

		unsigned int PathHeap_PercolateUp(PathHeap* a1, unsigned int a2)
		{
			return utils::hook::invoke<unsigned int>(0x3F7620_b, a1, a2);
		}

		game::pathnode_t* Path_OpenSet_Front(PathHeap* openSet)
		{
			return utils::hook::invoke<game::pathnode_t*>(0x3EBD60_b, openSet);
		}

		game::pathnode_t* PathHeap_PercolateDown(PathHeap* heap, unsigned int node)
		{
			return utils::hook::invoke<game::pathnode_t*>(0x3E91A0_b, heap, node);
		}

		bool Path_IsNodeUseableBy(const game::pathnode_t* pNode, unsigned int entNum)
		{
			return utils::hook::invoke<bool>(0x3EB500_b, pNode, entNum);
		}

		char Path_NodeOccupiedByOther(int searchEntNum, const game::pathnode_t* node)
		{
			return utils::hook::invoke<char>(0x3EBD00_b, searchEntNum, node);
		}

		int Path_OpenSet_Update(PathHeap* openSet, game::pathnode_t* node, game::pathnode_t* parent, const float cost)
		{
			return utils::hook::invoke<int>(0x3EBD90_b, openSet, node, parent, cost);
		}

		int Path_GeneratePath(path_s* pPath, const float* vStartPos, const float* vGoalPos,
			game::pathnode_t* pNodeFrom, game::pathnode_t* pNodeTo, int bIncludeGoalPos, int bAllowNegotiationLinks, int useChokePoints)
		{
			return utils::hook::invoke<int>(0x3EAA60_b, pPath, vStartPos, vGoalPos, pNodeFrom, pNodeTo, bIncludeGoalPos, bAllowNegotiationLinks, useChokePoints);
		}

		int Path_AStarAlgorithm_CustomSearchInfo_FindPathAlien_(PathFindInput* pathFindInput, int eTeam,
			int bIncludeGoalInPath, CustomSearchInfo_FindPathAlien* custom)
		{
			float* g_pathAttemptGoalPos = reinterpret_cast<float*>(0x718CDA0_b);
			int* level_iSearchFrame = reinterpret_cast<int*>(0x7361A68_b);

			game::pathnode_t* pNodeTo; // rbx
			game::pathnode_t* v18; // rdi
			int v20; // ecx
			float v21; // xmm0_4
			float v22; // xmm0_4
			float v23; // xmm1_4
			float v24; // xmm0_4
			float pos[3]{};

			const auto path_heap = std::make_unique<PathHeap>();

			auto pPath = pathFindInput->pPath;
			auto vGoalPos = pathFindInput->vGoalPos;

			if (pathFindInput->pPath)
			{
				pathFindInput->pPath->ownerEntNum = pathFindInput->searchEntNum;
			}

			if (eTeam > TEAM_AXIS)
			{
				eTeam = TEAM_AXIS;
			}

			auto v10 = eTeam - 1;
			auto v35 = eTeam - 1;

			if (pathFindInput->vGoalPos != nullptr)
			{
				g_pathAttemptGoalPos[0] = pathFindInput->vGoalPos[0];
				g_pathAttemptGoalPos[1] = pathFindInput->vGoalPos[1];
				g_pathAttemptGoalPos[2] = pathFindInput->vGoalPos[2];
				pos[0] = pathFindInput->vGoalPos[0];
				pos[1] = pathFindInput->vGoalPos[1];
				pos[2] = pathFindInput->vGoalPos[2];
				pathnode_t_LocalizePosToParent(pathFindInput->pNodeFrom, pos);
			}
			else
			{
				g_pathAttemptGoalPos[0] = 0.f;
				g_pathAttemptGoalPos[1] = 0.f;
				g_pathAttemptGoalPos[2] = 0.0f;
			}

			++ * level_iSearchFrame;

			const auto pNodeFrom = pathFindInput->pNodeFrom;
			pNodeFrom->transient.iSearchFrame = *level_iSearchFrame;
			pNodeFrom->transient.pParent = nullptr;
			pNodeFrom->transient.fCost = 0.f;

			const auto from_idx = static_cast<std::uint16_t>(Path_ConvertNodeToIndex(pNodeFrom));
			path_heap->index[from_idx] = 0;
			path_heap->nodes[0] = from_idx;

			if (++path_heap->size > 1)
			{
				PathHeap_PercolateUp(path_heap.get(), 0);
			}

			const auto scr_const_chyron_message1 = game::SL_GetString("ladder_up", 0);
			const auto scr_const_chyron_message2 = game::SL_GetString("ladder_down", 0);

			auto v13 = 0;
			if (Path_OpenSet_Front(path_heap.get()))
			{
				while (1)
				{
					pNodeTo = Path_OpenSet_Front(path_heap.get());
					if (pNodeTo == custom->m_pNodeTo)
					{
						break;
					}

					path_heap->index[path_heap->nodes[0]] = 0u;
					const auto node_idx = path_heap->nodes[--path_heap->size];
					path_heap->index[node_idx] = 0;
					path_heap->nodes[0] = node_idx;

					if (path_heap->size > 1)
					{
						PathHeap_PercolateDown(path_heap.get(), 0);
					}

					auto v17 = 0;

					if (pNodeTo->dynamic.wLinkCount > 0)
					{
						do
						{
							const auto animscript_to = pNodeTo->constant.animscript;
							if (pathFindInput->badplacePercent < 1.0 || !pNodeTo->constant.Links->ubBadPlaceCount[v10])
							{
								v18 = Path_ConvertIndexToNode(pNodeTo->constant.Links[v13].nodeNum);
								if (Path_IsNodeUseableBy(pNodeTo, pathFindInput->searchEntNum))
								{
									if (!is_traverse_begin_or_jump_node(pNodeTo->constant.type) || !is_traverse_end_or_jump_node(v18->constant.type)
										|| (1 //pathFindInput->bAllowJumpLinks ( this should always be 1 in aliens )
											|| pNodeTo->constant.type == NODE_DONTLINK || v18->constant.type == NODE_DONTLINK
											&& (pNodeTo->constant.target != v18->constant.targetname || pathFindInput->bAllowNegotiationLinks
												&& (pathFindInput->bAllowLadderNodes || (animscript_to != scr_const_chyron_message1) && animscript_to != scr_const_chyron_message2))
											&& ((v20 = pathFindInput->searchEntNum, v20 == 2047)
												|| (!pNodeTo->dynamic.wOverlapCount || !Path_NodeOccupiedByOther(v20, pNodeTo))
												&& (!v18->dynamic.wOverlapCount || !Path_NodeOccupiedByOther(pathFindInput->searchEntNum, v18)))))
									{
										if (v18->transient.iSearchFrame == *level_iSearchFrame)
										{
											v21 = (CustomSearchInfo_FindPathAlien_GetLinkCost(custom, pNodeTo, v17) + pNodeTo->transient.fCost)
												+ v18->transient.___u6.nodeCost;
											if (v21 < v18->transient.fCost)
											{
												Path_OpenSet_Update(path_heap.get(), v18, pNodeTo, v21);
											}
										}
										else
										{
											v18->transient.iSearchFrame = *level_iSearchFrame;
											v22 = CustomSearchInfo_FindPathAlien_GetNodeCost(custom, v18, pos);
											v18->transient.___u6.nodeCost = v22;
											v23 = pathFindInput->badplacePercent;

											if (v23 < 1.f && pNodeTo->constant.Links->ubBadPlaceCount[v35 + v13 * 12])
												v18->transient.___u6.nodeCost = (v23 * 256000.f) + v22;

											v18->transient.fHeuristic = sqrtf(
												(((pos[1] - v18->constant.vLocalOrigin[1])
													* (pos[1] - v18->constant.vLocalOrigin[1]))
													+ ((pos[0] - v18->constant.vLocalOrigin[0])
														* (pos[0] - v18->constant.vLocalOrigin[0])))
												+ ((pos[2] - v18->constant.vLocalOrigin[2])
													* (pos[2] - v18->constant.vLocalOrigin[2])));

											v24 = (CustomSearchInfo_FindPathAlien_GetLinkCost(custom, pNodeTo, v17) + pNodeTo->transient.fCost)
												+ v18->transient.___u6.nodeCost;

											v18->transient.pParent = pNodeTo;
											v18->transient.fCost = v24;

											auto v25 = static_cast<std::uint16_t>(Path_ConvertNodeToIndex(v18));
											auto v26 = static_cast<std::uint16_t>(path_heap->size);

											path_heap->index[v25] = v26;
											path_heap->nodes[v26] = v25;

											if (++path_heap->size > 1)
											{
												PathHeap_PercolateUp(path_heap.get(), v26);
											}
										}
									}
								}
								v10 = v35;
							}
							++v17;
							++v13;
						} while (v17 < pNodeTo->dynamic.wLinkCount);
						v13 = 0;
					}
					pNodeTo->transient.pPrevOpen = nullptr;

					if (!Path_OpenSet_Front(path_heap.get()))
					{
						return 0;
					}
				}

				if (pPath)
				{
					return Path_GeneratePath(
						pPath,
						pathFindInput->vStartPos,
						vGoalPos,
						pNodeFrom,
						pNodeTo,
						bIncludeGoalInPath,
						pathFindInput->bAllowNegotiationLinks,
						pathFindInput->useChokePoints);
				}
			}
			else
			{
				return 0;
			}

			return 1;
		}

		int Path_FindPathAlien(PathFindInput* pathFindInput)
		{
			CustomSearchInfo_FindPathAlien custom{};

			auto result = Path_CheckFromAndToNodes(pathFindInput);
			if (result)
			{
				CustomSearchInfo_FindPathAlien_CustomSearchInfo_FindPathAlien(&custom, pathFindInput);
				custom.m_pNodeTo = pathFindInput->pNodeTo;
				custom.startPos[0] = pathFindInput->vStartPos[0];
				custom.startPos[1] = pathFindInput->vStartPos[1];
				custom.startPos[2] = pathFindInput->vStartPos[2];
				result = Path_AStarAlgorithm_CustomSearchInfo_FindPathAlien_(pathFindInput, pathFindInput->pPath->eTeam, 1, &custom);
			}

			if (pathFindInput->pNodeTo != nullptr)
			{
				if (is_jump_node(pathFindInput->pNodeFrom->constant.type) || is_jump_node(pathFindInput->pNodeTo->constant.type))
				{
					printf("findpath: %i %i: %i\n", pathFindInput->pNodeFrom->constant.type,
						pathFindInput->pNodeTo->constant.type, pathFindInput->pPath->wNegotiationStartNode);
				}
			}


			return result;
		}

		int path_find_path_stub(PathFindInput* input, scripted_agent_info_t* info)
		{
			if ((info->pathTraversalPermissions & SA_PATH_ALLOW_JUMP) != 0)
			{
				bAllowJumpLinks = true;
				return Path_FindPathAlien(input);
			}
			else
			{
				return utils::hook::invoke<int>(0x3EA9C0_b, input);
			}
		}

		void path_find_path_stub_asm(utils::hook::assembler& a)
		{
			a.push(rax);
			a.pushad64();
			a.mov(rdx, rbx);
			a.call_aligned(path_find_path_stub);
			a.mov(qword_ptr(rsp, 0x80), rax);
			a.popad64();
			a.pop(rax);

			a.jmp(0x55C243_b);
		}

		const char* node_types[] =
		{
			"Error",
			"Path",
			"Cover Stand",
			"Cover Crouch",
			"Cover Crouch Window",
			"Cover Prone",
			"Cover Right",
			"Cover Left",
			"Cover Wide Right",
			"Cover Wide Left",
			"Cover Multi",
			"Ambush",
			"Exposed",
			"Conceal Stand",
			"Conceal Crouch",
			"Conceal Prone",
			"Door",
			"Door Interior",
			"Scripted",
			"Begin",
			"End",
			"Turret",
			"Guard",
			"Path 3D",
			"Cover Up 3D",
			"Cover Right 3D",
			"Cover Left 3D",
			"Exposed 3D",
			"Scripted 3D",
			"Begin 3D",
			"End 3D",
			"",
			"Jump",
			"Jump Attack",
		};

		bool check_traverse_node(const game::pathnode_t* a1, const game::pathnode_t* a2)
		{
			if (!scr_enable_jump_nodes->current.enabled)
			{
				return is_traverse_begin_node(a1->constant.type) && is_traverse_end_node(a2->constant.type) &&
					(a1->constant.target == a2->constant.targetname);
			}
			else
			{
				return is_traverse_begin_or_jump_node(a1->constant.type) && is_traverse_end_or_jump_node(a2->constant.type) &&
					(a1->constant.target == a2->constant.targetname || (is_jump_node(a1->constant.type) && is_jump_node(a2->constant.type)));
			}
		}

		void path_generate_path_stub(utils::hook::assembler& a)
		{
			const auto do_traverse = a.newLabel();

			a.pushad64();
			a.mov(rcx, rdi);
			a.mov(rdx, rsi);
			a.call_aligned(check_traverse_node);

			a.test(al, al);
			a.jnz(do_traverse);

			a.popad64();
			a.jmp(0x3EAD2A_b);

			a.bind(do_traverse);
			a.popad64();
			a.jmp(0x3EAD0A_b);
		}
		
		float distance(float* a, float* b)
		{
			return std::sqrtf((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]));
		}

		game::pathnode_tree_t* allocate_tree(game::PathData* asset)
		{
			++asset->nodeTreeCount;
			return reinterpret_cast<game::pathnode_tree_t*>(
				game::mp::Hunk_AllocAlignInternal(sizeof(game::pathnode_tree_t), 4));
		}

		game::pathnode_tree_t* build_node_tree(game::PathData* asset, unsigned short* node_indexes, const int num_nodes)
		{
			if (num_nodes < 4)
			{
				const auto result = allocate_tree(asset);
				result->axis = -1;
				result->u.s.nodeCount = num_nodes;
				result->u.s.nodes = node_indexes;
				return result;
			}

			game::vec2_t maxs{};
			game::vec2_t mins{};

			const auto start_node = &asset->nodes[*node_indexes];
			maxs[0] = start_node->constant.vLocalOrigin[0];
			mins[0] = maxs[0];
			maxs[1] = start_node->constant.vLocalOrigin[1];
			mins[1] = maxs[1];

			for (auto i = 1; i < num_nodes; i++)
			{	
				for (auto axis = 0; axis < 2; axis++)
				{
					const auto node = &asset->nodes[node_indexes[i]];
					const auto value = node->constant.vLocalOrigin[axis];
					if (mins[axis] <= value)
					{
						if (value > maxs[axis])
						{
							maxs[axis] = value;
						}
					}
					else
					{
						mins[axis] = value;
					}
				}
			}

			const auto axis = (maxs[1] - mins[1]) > (maxs[0] - mins[0]);
			if ((maxs[axis] - mins[axis]) > 192.f)
			{
				const auto dist = (maxs[axis] + mins[axis]) * 0.5f;
				auto left = 0;

				for (auto right = num_nodes - 1; ; --right)
				{
					while (dist > asset->nodes[node_indexes[left]].constant.vLocalOrigin[axis])
					{
						++left;
					}

					while (asset->nodes[node_indexes[right]].constant.vLocalOrigin[axis] > dist)
					{
						--right;
					}

					if (left >= right)
					{
						break;
					}

					const auto swap_node = node_indexes[left];
					node_indexes[left] = node_indexes[right];
					node_indexes[right] = swap_node;
					++left;
				}

				while (2 * left < num_nodes &&
					asset->nodes[node_indexes[left]].constant.vLocalOrigin[axis] == dist)
				{
					++left;
				}

				while (2 * left < num_nodes &&
					asset->nodes[node_indexes[left - 1]].constant.vLocalOrigin[axis] == dist)
				{
					--left;
				}

				game::pathnode_tree_t* child[2]{};
				child[0] = build_node_tree(asset, node_indexes, left);
				child[1] = build_node_tree(asset, &node_indexes[left], num_nodes - left);
				const auto result = allocate_tree(asset);
				result->axis = axis;
				result->dist = dist;
				result->u.child[0] = child[0];
				result->u.child[1] = child[1];
				
				return result;
			}
			
			const auto result = allocate_tree(asset);
			result->axis = -1;
			result->u.s.nodeCount = num_nodes;
			result->u.s.nodes = node_indexes;
			return result;
		}
		
		void path_init_stub()
		{
			// Path_Init
			utils::hook::invoke<void>(0x3F8370_b);

			if (game::VirtualLobby_Loaded() || !scr_load_csv_paths->current.enabled)
			{
				return;
			}

			// load new paths with zt util to parse bot warfare from disk
			std::string buffer;

			auto mapname = game::Dvar_FindVar("mapname");
			
			if (const auto file_path = std::format("maps/mp/{}_wp.csv", mapname->current.string); filesystem::read_file(file_path, &buffer))
			{
				console::info("Loading paths '%s' from disk\n", file_path.data());
				
				auto table = utils::csv::parser(buffer);

				auto* asset = path_allocator.allocate<game::PathData>(); // stringtable allocator lul

				asset->name = path_allocator.duplicate_string(std::string(mapname->current.string));
				
				if (table.get_num_rows() <= 0)
				{
					return;
				}
				
				const auto rows = table.get_rows();
				asset->nodeCount = std::atoi(rows[0]->fields[0]);
				asset->nodes = path_allocator.allocate_array<game::pathnode_t>(asset->nodeCount);

				if (table.get_num_rows() < static_cast<int>(asset->nodeCount))
				{
					console::error("Less than asset->nodeCount + 1 (%i) rows", asset->nodeCount);
					return;
				}

				for (auto i = 0u; i < asset->nodeCount; i++)
				{
					const auto row = rows[i + 1];
					const auto node = &asset->nodes[i];
					node->constant.type = 1;

					if (row->num_fields < 4)
					{
						console::error("Not enough fields for node num %i (must be origin,links,type,angles,...)", i);
						return;
					}

					const auto origin_str = utils::string::split(row->fields[0], ' ');
					if (origin_str.size() == 3)
					{
						node->constant.vLocalOrigin[0] = static_cast<float>(std::atof(origin_str[0].data()));
						node->constant.vLocalOrigin[1] = static_cast<float>(std::atof(origin_str[1].data()));
						node->constant.vLocalOrigin[2] = static_cast<float>(std::atof(origin_str[2].data()));
					}

					auto field = row->fields[2];
					auto field_str = path_allocator.duplicate_string(field);
					if (waypoint_types.contains(field_str))
					{
						node->constant.type = waypoint_types.at(field_str);
					}

					const auto angles_str = utils::string::split(row->fields[3], ' ');
					if (angles_str.size() == 3)
					{
						node->constant.___u9.angles[0] = static_cast<float>(std::atof(angles_str[1].data()));
						node->constant.___u9.angles[1] = static_cast<float>(std::atof(angles_str[0].data()));
						node->constant.___u9.angles[2] = static_cast<float>(std::atof(angles_str[2].data()));
					}

					const auto links_str = utils::string::split(row->fields[1], ' ');
					node->constant.totalLinkCount = static_cast<unsigned short>(links_str.size());
					node->constant.Links = path_allocator.allocate_array<game::pathlink_s>(links_str.size());
					for (auto o = 0; o < node->constant.totalLinkCount; o++)
					{
						const auto num = std::atoi(links_str[o].data());
						node->constant.Links[o].nodeNum = static_cast<unsigned short>(num);
					}
				}

				for (auto i = 0u; i < asset->nodeCount; i++)
				{
					const auto node = &asset->nodes[i];
					for (auto o = 0; o < node->constant.totalLinkCount; o++)
					{
						const auto linked_num = node->constant.Links[o].nodeNum;
						if (linked_num >= asset->nodeCount)
						{
							console::error("Node link num out of bounds");
							return;
						}

						const auto linked = &asset->nodes[linked_num];
						node->constant.Links[o].negotiationLink = 1;
						node->constant.Links[o].fDist = 
							distance(node->constant.vLocalOrigin, linked->constant.vLocalOrigin);
					}
				}

				const auto node_indexes = path_allocator.allocate_array<unsigned short>(asset->nodeCount);
				for (auto i = 0u; i < asset->nodeCount; i++)
				{
					node_indexes[i] = static_cast<unsigned short>(i);
				}

				asset->nodeTree = build_node_tree(asset, node_indexes, asset->nodeCount);

				console::debug("new node count is %d\n", asset->nodeCount);

				*game::pathdata = *asset;
			}
		}
	}

	class component final : public component_interface
	{
	public:
		void post_unpack() override
		{
			if (game::environment::is_sp())
			{
				return;
			}

			scr_enable_jump_nodes = dvars::register_bool("scr_enableJumpNodes", false, game::DVAR_FLAG_REPLICATED, "enable jump nodes");

			// implement jump nodes from iw6
			utils::hook::inject(0x3F5F03_b + 3, node_types);
			utils::hook::inject(0x3F66A0_b + 3, node_types);
			utils::hook::set<std::uint8_t>(0x3F66E7_b + 2, node_type_end);

			utils::hook::jump(0x3EACE0_b, utils::hook::assemble(path_generate_path_stub), true);

			//utils::hook::far_jump<0x140000000>(0x55C23E_b, utils::hook::assemble(path_find_path_stub_asm));
			//utils::hook::jump(0x3EACE0_b, 0x3EAD27_b);

			gsc::function::add("markdangerousnodes", mark_dangerous_nodes);
			gsc::function::add("markdangerousnodesintrigger", mark_dangerous_nodes_in_trigger);
			
			// add bot warfare CSV loading on map loading
			scr_load_csv_paths = dvars::register_bool("scr_loadCsvPaths", false, game::DVAR_FLAG_NONE,
				"Replace the map's path nodes with maps/mp/<mapname>_wp.csv when it exists");
			utils::hook::call(0x420911_b, path_init_stub);
		}
	};
}

REGISTER_COMPONENT(pathnodes::component)
