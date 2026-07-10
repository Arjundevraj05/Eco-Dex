"use client";
import React, { useEffect, useState } from "react";
import Image from "next/image";
import CountUp from "react-countup";
import { Coins, Leaf } from "lucide-react";
import { motion } from "framer-motion";
import { Stagger, StaggerItem } from "@/components/AnimatedSection";
import { cardHover, tapScale } from "@/lib/motion";

interface WasteItem {
  isBiodegradable: boolean;
  Class: "PLASTIC" | "METAL" | "PAPER" | "CARDBOARD" | "GLASS";
}

function ImpactCard({
  gradient,
  iconBg,
  icon,
  title,
  value,
  suffix,
  titleColor,
  valueColor,
}: {
  gradient: string;
  iconBg: string;
  icon: React.ReactNode;
  title: string;
  value: number;
  suffix?: string;
  titleColor: string;
  valueColor: string;
}) {
  return (
    <motion.div
      className={`flex h-36 w-full max-w-sm cursor-pointer flex-row items-center rounded-lg bg-gradient-to-b ${gradient} p-[2px] sm:w-72 sm:max-w-none`}
      whileHover={cardHover}
      whileTap={tapScale}
    >
      <div className="flex h-full w-full items-center rounded-lg bg-gray-50 p-4 sm:p-5">
        <div className={`shrink-0 rounded-full bg-gradient-to-b p-2.5 sm:p-3 ${iconBg}`}>
          {icon}
        </div>
        <div className="ml-3 min-w-0 flex-1 sm:ml-4">
          <p className={`text-base font-semibold leading-tight sm:text-xl ${titleColor}`}>{title}</p>
          <p className={`mt-1 text-2xl font-bold sm:mt-2 sm:text-3xl ${valueColor}`}>
            <CountUp end={value} duration={2.5} />
            {suffix && <span className="text-lg sm:text-xl"> {suffix}</span>}
          </p>
        </div>
      </div>
    </motion.div>
  );
}

const Status1 = () => {
  const [biodegradableCount, setBiodegradableCount] = useState(0);
  const [totalWasteCount, setTotalWasteCount] = useState(0);
  const [tokensEarned, setTokensEarned] = useState(0);
  const [carbonReduced, setCarbonReduced] = useState(0);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchReports = async () => {
      try {
        const response = await fetch("/api/users/current", { credentials: "include" });
        if (!response.ok) {
          throw new Error("Failed to fetch records");
        }
        const data: WasteItem[] = await response.json();

        setTotalWasteCount(data.length);

        const biodegradableItems = data.filter((item) => item.isBiodegradable);
        setBiodegradableCount(biodegradableItems.length);

        const counts = data.reduce(
          (acc, item) => {
            acc[item.Class] = (acc[item.Class] || 0) + 1;
            return acc;
          },
          { PLASTIC: 0, METAL: 0, PAPER: 0, CARDBOARD: 0, GLASS: 0 }
        );

        setTokensEarned(
          counts.PLASTIC * 2 + counts.METAL * 3 + counts.PAPER * 1 + counts.CARDBOARD * 2 + counts.GLASS * 2
        );

        setCarbonReduced(
          counts.PLASTIC * 6 + counts.METAL * 10 + counts.PAPER * 4 + counts.CARDBOARD * 5 + counts.GLASS * 3
        );
      } catch (err) {
        console.error("Error fetching data:", err);
      } finally {
        setLoading(false);
      }
    };

    fetchReports();
  }, []);

  if (loading) {
    return (
      <div className="mb-6 mt-10 flex justify-center">
        <p className="text-gray-500">Loading impact stats...</p>
      </div>
    );
  }

  return (
    <Stagger className="mb-6 mt-6 flex flex-wrap justify-center gap-3 sm:mt-10 sm:gap-4">
      <StaggerItem>
      <ImpactCard
        gradient="from-green-300 to-green-500"
        iconBg="from-green-100 to-green-200"
        icon={<Image src="/icons/biodegradable.svg" width={50} height={50} alt="Biodegradable" />}
        title="Biodegradable"
        value={biodegradableCount}
        titleColor="text-green-800"
        valueColor="text-green-900"
      />
      </StaggerItem>
      <StaggerItem>
      <ImpactCard
        gradient="from-red-300 to-red-500"
        iconBg="from-red-100 to-red-200"
        icon={<Image src="/icons/nonbiodegradable.svg" width={50} height={50} alt="Non-Biodegradable" />}
        title="Non-Biodegradable"
        value={totalWasteCount - biodegradableCount}
        titleColor="text-red-800"
        valueColor="text-red-900"
      />
      </StaggerItem>
      <StaggerItem>
      <ImpactCard
        gradient="from-gray-300 to-gray-500"
        iconBg="from-gray-100 to-gray-200"
        icon={<Leaf className="h-10 w-10 text-gray-700 sm:h-12 sm:w-12" />}
        title="Carbon Reduced"
        value={carbonReduced}
        suffix="kg"
        titleColor="text-gray-700"
        valueColor="text-gray-800"
      />
      </StaggerItem>
      <StaggerItem>
      <ImpactCard
        gradient="from-yellow-300 to-yellow-500"
        iconBg="from-yellow-100 to-yellow-200"
        icon={<Coins className="h-10 w-10 text-yellow-600 sm:h-12 sm:w-12" />}
        title="Tokens Earned"
        value={tokensEarned}
        titleColor="text-yellow-800"
        valueColor="text-yellow-900"
      />
      </StaggerItem>
    </Stagger>
  );
};

export default Status1;
