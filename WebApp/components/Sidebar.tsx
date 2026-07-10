'use client';
import React, { useState, useEffect } from 'react';
import Image from 'next/image';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { AnimatePresence, motion } from 'framer-motion';
import {
  HomeIcon,
  DocumentReportIcon,
  MapIcon,
  AdjustmentsIcon,
  ShieldCheckIcon,
  MenuAlt2Icon,
  XIcon,
  UserCircleIcon,
} from '@heroicons/react/outline';
import LogoutButton from './LogoutButton';
import { fetchUsername } from '../utils/fetchUsername';

const isDemoMode = process.env.NEXT_PUBLIC_DEMO_MODE === 'true';

const navigation = [
  { name: 'Home', icon: HomeIcon, href: '/' },
  { name: 'Analysis', icon: DocumentReportIcon, href: '/reports' },
  { name: 'Map', icon: MapIcon, href: '/map' },
  { name: 'Manual Override', icon: AdjustmentsIcon, href: '/livecam' },
  { name: 'Security', icon: ShieldCheckIcon, href: '/override' },
];

const Sidebar: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [username, setUsername] = useState<string | null>(null);
  const pathname = usePathname();

  useEffect(() => {
    const getUsername = async () => {
      const fetchedUsername = await fetchUsername();
      setUsername(fetchedUsername);
    };
    getUsername();
  }, []);

  useEffect(() => {
    setIsOpen(false);
  }, [pathname]);

  const isActive = (href: string) =>
    href === '/' ? pathname === '/' : pathname.startsWith(href);

  return (
    <>
      <div className="fixed top-0 left-0 right-0 z-[1120] flex h-14 items-center justify-between bg-gray-50 px-4 shadow-md md:hidden">
        <button
          onClick={() => setIsOpen(!isOpen)}
          aria-label="Toggle Sidebar"
          className="focus:outline-none"
        >
          {isOpen ? <XIcon className="h-6 w-6" /> : <MenuAlt2Icon className="h-6 w-6" />}
        </button>
        <div className="flex items-center">
          <Image src="/icons/logo_main.svg" width={34} height={34} alt="logo" />
          <span className="ml-2 font-bold font-stacker text-lg sm:text-xl">Eco-Dex</span>
        </div>
        <div className="w-6" />
      </div>

      <AnimatePresence>
        {isOpen && (
          <motion.div
            className="fixed inset-0 z-[1100] bg-black/50 md:hidden"
            onClick={() => setIsOpen(false)}
            aria-hidden="true"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.2 }}
          />
        )}
      </AnimatePresence>

      <aside
        className={`fixed left-0 z-[1110] w-52 bg-gray-50 text-black border-r-2 shadow-lg
          top-14 bottom-0 md:top-0
          transition-transform duration-300 ease-[cubic-bezier(0.22,1,0.36,1)]
          ${isOpen ? 'translate-x-0' : '-translate-x-full'}
          md:translate-x-0`}
      >
        <div className="flex h-full flex-col overflow-y-auto p-4 sm:p-6">
          <div className="mb-6 hidden items-center border-b border-gray-300 pb-4 md:flex">
            <Image src="/icons/logo_main.svg" width={34} height={34} alt="logo" />
            <span className="ml-2 text-lg font-stacker font-bold">Eco-Dex</span>
          </div>

          <nav className="flex-1">
            <ul className="space-y-1">
              {navigation.map((item, index) => (
                <motion.li
                  key={item.name}
                  initial={{ opacity: 0, x: -12 }}
                  animate={{ opacity: 1, x: 0 }}
                  transition={{ delay: index * 0.05, duration: 0.3 }}
                >
                  <Link
                    href={item.href}
                    onClick={() => setIsOpen(false)}
                    className={`flex items-center w-full px-3 sm:px-4 py-2.5 sm:py-3 text-medium font-medium transition-all duration-200 rounded-md ${
                      isActive(item.href)
                        ? 'bg-green-100 text-green-600 shadow-sm'
                        : 'text-gray-700 hover:bg-gray-100 hover:text-green-600'
                    }`}
                  >
                    <item.icon
                      className={`h-5 w-5 sm:h-6 sm:w-6 shrink-0 ${
                        isActive(item.href) ? 'text-green-600' : 'text-gray-500'
                      }`}
                    />
                    <span className="ml-2 text-sm leading-tight">{item.name}</span>
                  </Link>
                </motion.li>
              ))}
            </ul>
          </nav>

          {isDemoMode ? (
            <div className="mb-4 w-full rounded-lg bg-green-100 py-3 text-center text-sm font-medium text-green-800">
              Demo Mode
            </div>
          ) : (
            <LogoutButton />
          )}

          <div className="mt-6 flex items-center border-t border-gray-300 pt-4 space-x-3 sm:space-x-4">
            <UserCircleIcon className="h-9 w-9 sm:h-10 sm:w-10 text-gray-500 shrink-0" />
            <div className="min-w-0">
              <p className="truncate text-sm font-medium">{username ? username : 'Guest'}</p>
              <p className="text-xs text-gray-500">Operator</p>
            </div>
          </div>
        </div>
      </aside>
    </>
  );
};

export default Sidebar;
