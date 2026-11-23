#include <std_include.hpp>
#include "loader/component_loader.hpp"

#include "game/game.hpp"
#include "game/dvars.hpp"

#include "gsc/script_extension.hpp"

#include <utils/hook.hpp>

game::dvar_t* g_useOldCollisionMask = nullptr;

namespace aliens
{
#define USE_MODEL_COLLISION_BOUNDS_FLAG 0x8
	
	bool G_IsAliensMode()
	{
		static auto* g_gametype = game::Dvar_FindVar("g_gametype");
		assert(g_gametype && g_gametype->current.string);
		return strcmp(g_gametype->current.string, "aliens") == 0;
	}

	constexpr unsigned char aliensPriorityMap[game::HITLOC_COUNT] =
	{
		1,   // HITLOC_NONE

		// Head
		3,   // HELMET
		3,   // HEAD
		3,   // NECK

		// Torso
		3,   // TORSO_UPR
		3,   // TORSO_LWR

		// Right arm
		3,   // R_ARM_UPR
		3,   // R_ARM_LWR
		3,   // R_HAND

		// Left arm
		3,   // L_ARM_UPR
		3,   // L_ARM_LWR
		3,   // L_HAND

		// Right leg
		3,   // R_LEG_UPR
		3,   // R_LEG_LWR
		3,   // R_FOOT

		// Left leg
		3,   // L_LEG_UPR
		3,   // L_LEG_LWR
		3,   // L_FOOT

		0,   // GUN
		201, // SHIELD
		3,   // ARMOR
		3,   // SOFT
	};
	
	namespace
	{
		void* sv_linkentity_fix_hook()
		{
			return utils::hook::assemble([](utils::hook::assembler& a)
			{
				const auto l_jump_568590 = a.newLabel();
				const auto l_jump_568548 = a.newLabel();

				a.mov(rcx, rbx); // mov     rcx, rbx
				a.call(reinterpret_cast<void*>(0x4E99D0_b));
				a.test(rax, rax); // test    rax, rax
				a.jz(l_jump_568590);
				a.movzx(ecx, byte_ptr(rbx, 0x102)); // movzx   ecx, byte ptr [rbx+102h]

				a.test(asmjit::x86::cl, 6);
				a.jz(l_jump_568590);

				a.test(asmjit::x86::cl, USE_MODEL_COLLISION_BOUNDS_FLAG);
				a.jnz(l_jump_568548);

				a.test(asmjit::x86::cl, 2);
				a.jz(l_jump_568548);

				a.jmp(reinterpret_cast<void*>(0x568501_b));

				a.bind(l_jump_568590);
				a.jmp(reinterpret_cast<void*>(0x568590_b));

				a.bind(l_jump_568548);
				a.jmp(reinterpret_cast<void*>(0x568548_b));
			});
		}

		void* sv_point_trace_to_entity_fix_hook()
		{
			return utils::hook::assemble([](utils::hook::assembler& a)
			{
				const auto l_jump_5691D2 = a.newLabel();
				const auto l_jump_568EDC = a.newLabel();
				const auto l_jump_568F10 = a.newLabel();

				a.movzx(eax, byte_ptr(rbx, 0x102)); // movzx   eax, byte ptr [rbx+102h]

				a.test(asmjit::x86::al, 6);
				a.jz(l_jump_5691D2);

				a.test(asmjit::x86::al, USE_MODEL_COLLISION_BOUNDS_FLAG);
				a.jnz(l_jump_568F10);

				a.test(asmjit::x86::al, 2);
				a.jz(l_jump_568EDC);

				a.jmp(reinterpret_cast<void*>(0x568ED1_b));

				a.bind(l_jump_5691D2);
				a.jmp(reinterpret_cast<void*>(0x5691D2_b));

				a.bind(l_jump_568EDC);
				a.jmp(reinterpret_cast<void*>(0x568EDC_b));

				a.bind(l_jump_568F10);
				a.movsx(r12d, r15d);
				a.mov(ecx, r12d);
				a.call(0x4E9A30_b);
				a.mov(r14, rax);
				a.jmp(reinterpret_cast<void*>(0x568F10_b));
			});
		}

		int G_TraceCapsuleComplete_stub(const float* start, const float* end, const game::Bounds* bounds, 
			const short passEntityNum, const short passEntityNum1, int contentmask)
		{
			if (G_IsAliensMode() || g_useOldCollisionMask->current.enabled)
			{
				contentmask = 0x11;
			}

			return utils::hook::invoke<int>(0x4234C0_b, start, end, bounds, passEntityNum, passEntityNum1, contentmask);
		}

		void G_LocationalTraceAllowChildren_stub(game::trace_t* results, const float* start, const float* end, unsigned short passEntityNum, int contentmask, unsigned char* priorityMap, char a7)
		{
			if (G_IsAliensMode())
			{
				priorityMap = const_cast<unsigned char*>(aliensPriorityMap);
			}

			utils::hook::invoke<void>(0x421360_b, results, start, end, passEntityNum, contentmask, priorityMap, a7);
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

			{
				// fix tags for clients/agents
				const auto patch_ncs_tag_check = [](std::size_t nop_addr, std::size_t nop_size)
				{
					utils::hook::nop(nop_addr, nop_size);
					utils::hook::set<uint8_t>(nop_addr + nop_size, 0xEB);
				};
				patch_ncs_tag_check(0x465333_b, 8); // G_UpdateTagInfoOfChildren
				patch_ncs_tag_check(0x46520B_b, 6); // G_ClientDObjUpdate
				patch_ncs_tag_check(0x464297_b, 6); // EntLinkTo_Internal
				patch_ncs_tag_check(0x4514F8_b, 6); // Scr_SpawnLinkedFXInternal
				patch_ncs_tag_check(0x451407_b, 6); // playfxonweapon
				patch_ncs_tag_check(0x44AACE_b, 6); // Scr_KillFXOnWeapon
				patch_ncs_tag_check(0x44A3CF_b, 6); // Scr_KillFXOnTagForClient
				patch_ncs_tag_check(0x449D1C_b, 6); // Scr_KillFXOnTag
				patch_ncs_tag_check(0x44975E_b, 6); // Scr_PlayFxOnTagInternal
				patch_ncs_tag_check(0x446C24_b, 6); // Scr_PlayFXOnTag
				patch_ncs_tag_check(0x44909F_b, 6); // Scr_StopFXOnTagForClient
				patch_ncs_tag_check(0x4486EC_b, 6); // Scr_StopFXOnTag
			}

			g_useOldCollisionMask = dvars::register_bool("g_useOldCollisionMask", false, game::DVAR_FLAG_NONE, "Use old collision mask for Player_GetUseList");

			// patch Player_GetUseList trace call
			utils::hook::call(0x466EA4_b, G_TraceCapsuleComplete_stub);
			// patch priorityMap for Bullet_Trace
			utils::hook::call(0x3F53A4_b, G_LocationalTraceAllowChildren_stub);

			utils::hook::jump(0x5684DB_b, sv_linkentity_fix_hook(), true);
			utils::hook::jump(0x568EBE_b, sv_point_trace_to_entity_fix_hook(), true);
			gsc::method::add("scragentusemodelcollisionbounds", [](const game::scr_entref_t entref, const gsc::function_args&)
				-> scripting::script_value
			{
				if (entref.classnum != 0)
				{
					throw std::runtime_error("not an entity");
				}

				const auto ent = &game::mp::g_entities[entref.entnum];
				if (!ent->agent)
				{
					throw std::runtime_error(utils::string::va("entity %i is not an agent", entref.entnum));
				}

				if (!game::Agent_IsScripted(entref.entnum))
				{
					throw std::runtime_error(utils::string::va("entity %i is not a scripted agent", entref.entnum));
				}

				ent->r.svFlags |= USE_MODEL_COLLISION_BOUNDS_FLAG;
				game::SV_LinkEntity(ent);

				return {};
			});
		}
	};
}

REGISTER_COMPONENT(aliens::component)
