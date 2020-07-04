
#pragma once

#include <cstdint>
#include <type_traits>

#define RCC (*(reinterpret_cast<volatile RCC_t*>(RCC_t::BASE)))
#define FLASH (*(reinterpret_cast<volatile FLASH_t*>(FLASH_t::BASE)))

using frequency_t = uint32_t;

constexpr auto operator"" _MHz(unsigned long long f) { return f * 1'000'000; }

enum class XTAL_traits {
	XTAL,
	DIRECT
};

template<frequency_t freq, XTAL_traits t = XTAL_traits::XTAL>
struct XTAL {
	static constexpr frequency_t value = freq;
	static constexpr XTAL_traits type = t;
};

template<class T>
struct is_XTAL : std::false_type {};

template<frequency_t freq>
struct is_XTAL<XTAL<freq>> : std::true_type {};

struct STM32F334 {

	struct HSI : XTAL<8> {};

	struct busses {
		static constexpr auto AHB3 = 0x5000'0000;
		static constexpr auto AHB2 = 0x4800'0000;
		static constexpr auto AHB1 = 0x4002'0000;
		static constexpr auto APB2 = 0x4001'0000;
		static constexpr auto APB1 = 0x4000'0000;
	};

	struct RCC_t {
		static constexpr auto BASE = busses::AHB1 + 0x0000'3000;
		struct CR_t {
			uint32_t HSION : 1;
			uint32_t HSIRDY : 1;
			uint32_t : 1;
			uint32_t HSITRIM : 5;
			uint32_t HSICAL : 8;
			uint32_t HSEON : 1;
			uint32_t HSERDY : 1;
			uint32_t HSEBYP : 1;
			uint32_t CSSON : 1;
			uint32_t : 4;
			uint32_t PLLON : 1;
			uint32_t PLLRDY : 1;
		};
		enum class CFGR_SW : uint32_t {
			HSI,
			HSE,
			PLL
		};
		enum class CFGR_PLLSRC : uint32_t {
			HSI,
			HSE
		};
		enum class CFGR_PLLXTPRE : uint32_t {
			no,
			yes
		};
		enum class CFGR_MCO : uint32_t {
			DISABLE,
			RESERVED,
			LSI,
			LSE,
			SYSCLK,
			HSI,
			HSE,
			PLL
		};
		struct CFGR_t {
			CFGR_SW SW : 2;
			CFGR_SW SWS : 2;
			uint32_t HPRE : 4;
			uint32_t PPRE1 : 3;
			uint32_t PPRE2 : 3;
			uint32_t : 2;
			CFGR_PLLSRC PLLSRC : 1;
			CFGR_PLLXTPRE PLLXTPRE : 1;
			uint32_t PLLMUL : 4;
			uint32_t : 2;
			CFGR_MCO MCO : 3;
			uint32_t : 1;
			uint32_t MCOPRE : 3;
			uint32_t PLLNODIV : 1;
		};
		struct CFGR2_t {
			uint32_t PREDIV : 4;
			uint32_t ADC12PRES : 5;
		};

		CR_t CR;
		CFGR_t CFGR;
		uint32_t CIR;
		uint32_t APB2RSTR;
		uint32_t APB1RSTR;
		uint32_t AHBENR;
		uint32_t APB2ENR;
		uint32_t APB1ENR;
		uint32_t BDCR;
		uint32_t CSR;
		uint32_t AHBRSTR;
		CFGR2_t CFGR2;
	};

	struct FLASH_t {
		struct ACR_t {
			uint32_t LATENCY : 3;
			uint32_t HLFCYA : 1;
			uint32_t PRFTBE : 1;
			const uint32_t PRFTBS : 1;
		};

		ACR_t ACR;
		uint32_t KEYR;
		uint32_t OPTKEYR;
		uint32_t SR;
		uint32_t CR;
		uint32_t AR;
		const uint32_t OBR;
		uint32_t WRPR;
	};

	template<class xtal, frequency_t target = xtal::value>
	class default_clocks {
		static_assert(std::is_same_v<xtal, HSI> || is_XTAL<xtal>::value, "template parameter xtal has to be HSI or a direct specialization of struct XTAL");
		default_clocks() = delete;
		
		struct pll {
			uint32_t div;
			uint32_t mul;
		};
		constexpr pll calc_impl() {
			for (uint32_t d = 1; d <= 16; d++)
				for (uint32_t m = 2; m <= 16; m++)
					if (xtal::value / d * m == target)
						return { d, m };
			return {0,0};
		}
		constexpr pll calc() {
			constexpr pll ret = calc_impl();
			static_assert(ret.div != 0 && ret.mul != 0, "Could not determin divider and multiplier values for PLL");
			return ret;
		}

		public:
			static constexpr frequency_t SYSCLK = target;

			static void configure() {
				if constexpr (target < 24_MHz) {
					FLASH.LATENCY = 0;
				} else if (target < 48_MHz) {
					FLASH.LATENCY = 1;
				} else {
					FLASH.LATENCY = 2;
				}

				if constexpr (std::is_same_v<xtal, HSI> && target != xtal::value) {
					RCC.CR.HSION = 1;
					while (!RCC.CR.HSIRDY);
					RCC.CFGR.SW = RCC_t::CFGR_SW::HSI;
					RCC.CR.HSEON = 0;
					RCC.CR.PLLON = 0;
				} else {
					// xtal is either a XTAL specialization (ie HSE) or target freq is not xtal freq and needs to be PLL'ed
					if constexpr (std::is_same_v<xtal, HSI>) {
						RCC.CR.HSION = 1;
						while (!RCC.CR.HSIRDY);
						RCC.CFGR.SW = RCC_t::CFGR_SW::HSI;
						RCC.CR.HSEON = 0;
					} else {
						// xtal is HSE
						RCC.CR.HSEON = 1;

						if constexpr (xtal::type == XTAL_traits::DIRECT) {
							RCC.CR.HSEBYP = 1;
						}

						while (!RCC.CR.HSERDY);
						RCC.CR.HSION = 0;
						RCC.CFGR.SW = RCC_t::CFGR_SW::HSE;
					}

					RCC.CR.PLLON = 0;
					if constexpr (target != xtal::value) {
						// switch to pll
						while (RCC.CR.PLLRDY);

						RCC.CFGR.PLLMUL = calc().mul - 2;
						RCC.CFGR2.PREDIV = calc().div - 1;

						RCC.CR.PLLON = 1;
						while (!RCC.CR.PLLRDY);
					}
				}
			}
	};

};
