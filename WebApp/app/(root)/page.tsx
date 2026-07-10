"use client";

import { Leaf, Coins, Users } from 'lucide-react';
import HeaderBox from "@/components/HeaderBox";
import Status1 from "@/components/Status1";
import { FadeIn, Stagger, StaggerItem } from "@/components/AnimatedSection";
import { motion } from 'framer-motion';
import { cardHover, tapScale } from '@/lib/motion';
import React from "react";

function AnimatedGlobe() {
  return (
    <motion.div
      className="relative mx-auto mb-6 mt-4 h-24 w-24 sm:mb-8 sm:mt-6 sm:h-32 sm:w-32"
      animate={{ y: [0, -6, 0] }}
      transition={{ duration: 3, repeat: Infinity, ease: 'easeInOut' }}
    >
      <div className="absolute inset-0 animate-soft-pulse rounded-full bg-green-500 opacity-20" />
      <div className="absolute inset-2 rounded-full bg-green-400 opacity-30" />
      <div className="absolute inset-4 rounded-full bg-green-300 opacity-40" />
      <Leaf className="absolute inset-0 m-auto h-12 w-12 text-green-600 sm:h-16 sm:w-16" />
    </motion.div>
  );
}

const Page: React.FC = () => {
  return (
    <div className="flex min-h-screen w-full flex-col items-center bg-green-50 font-sans">
      <HeaderBox />
      <div className="mb-12 w-full max-w-4xl px-4 pb-12 text-center sm:px-6 md:mb-20">
        <FadeIn>
          <AnimatedGlobe />
        </FadeIn>

        <FadeIn delay={0.1}>
          <h1 className="mb-4 text-3xl font-bold tracking-tight text-gray-800 sm:mb-6 sm:text-4xl md:text-5xl lg:text-6xl">
            Revolutionize <br /><span className="text-green-600">Waste Management</span>
          </h1>
        </FadeIn>

        <FadeIn delay={0.18}>
          <p className="mx-auto mb-8 max-w-2xl text-base leading-relaxed text-gray-600 sm:text-lg md:text-xl">
            With Eco-Dex and RAG-ED, experience a smarter, greener approach to waste collection!
          </p>
        </FadeIn>

        <Stagger className="mb-10 grid grid-cols-1 gap-6 sm:grid-cols-2 md:grid-cols-3 md:gap-8">
          <StaggerItem>
            <FeatureCard
              icon={Leaf}
              title="Eco-Friendly"
              description="Contribute to a cleaner environment by reporting and collecting waste."
            />
          </StaggerItem>
          <StaggerItem>
            <FeatureCard
              icon={Coins}
              title="Earn Rewards"
              description="Get tokens for your contributions to waste management efforts."
            />
          </StaggerItem>
          <StaggerItem>
            <FeatureCard
              icon={Users}
              title="Community-Driven"
              description="Be part of a growing community committed to sustainable practices."
            />
          </StaggerItem>
        </Stagger>

        <FadeIn delay={0.2}>
          <section className="mx-auto w-full max-w-4xl overflow-hidden rounded-2xl bg-white p-6 shadow-lg sm:rounded-3xl sm:p-8 md:p-10 lg:max-w-6xl">
            <h2 className="mb-8 text-center text-2xl font-bold text-gray-800 sm:mb-10 sm:text-3xl md:text-4xl">
              Our Impact
            </h2>
            <Status1 />
          </section>
        </FadeIn>
      </div>
    </div>
  );
};

interface FeatureCardProps {
  icon: React.ElementType;
  title: string;
  description: string;
}

const FeatureCard: React.FC<FeatureCardProps> = ({ icon: Icon, title, description }) => {
  return (
    <motion.div
      whileHover={cardHover}
      whileTap={tapScale}
      className="flex h-full flex-col items-center rounded-xl bg-white p-6 text-center shadow-md transition-shadow duration-300 hover:shadow-xl sm:p-8"
    >
      <motion.div
        className="mb-4 rounded-full bg-green-100 p-3 sm:mb-6 sm:p-4"
        whileHover={{ rotate: [0, -8, 8, 0], transition: { duration: 0.4 } }}
      >
        <Icon className="h-8 w-8 text-green-600 sm:h-10 sm:w-10" />
      </motion.div>
      <h3 className="mb-3 text-lg font-semibold text-gray-800 sm:mb-4 sm:text-xl">{title}</h3>
      <p className="text-sm leading-relaxed text-gray-600 sm:text-base">{description}</p>
    </motion.div>
  );
};

export default Page;
